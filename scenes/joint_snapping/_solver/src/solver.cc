// A simple example of using the Ceres minimizer.
//
#include <stdio.h>
#include <sstream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "glog/logging.h"

#ifdef EMSCRIPTEN

#include "emscripten/emscripten.h"
#include "emscripten/bind.h"

using namespace emscripten;

#endif

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace std;


struct Mat4 {
  double a0,  a1,   a2,  a3,
         a4,  a5,   a6,  a7,
         a8,  a9,  a10, a11,
         a12, a13, a14, a15;
};


template<typename T>
inline void matToEigen(const Mat4& m, Eigen::Matrix<T,4,4>& m_out) {
    m_out << T(m.a0),  T(m.a4),  T(m.a8),  T(m.a12),
             T(m.a1),  T(m.a5),  T(m.a9),  T(m.a13),
             T(m.a2),  T(m.a6),  T(m.a10), T(m.a14),
             T(m.a3),  T(m.a7),  T(m.a11), T(m.a15);
//    m_out << T(m.a0),  T(m.a1),  T(m.a2),  T(m.a3),
//             T(m.a4),  T(m.a5),  T(m.a6),  T(m.a7),
//             T(m.a8),  T(m.a9),  T(m.a10), T(m.a11),
//             T(m.a12), T(m.a13), T(m.a14), T(m.a15);
} 


struct Point2D {
  double x, y;
  Point2D() {}
  Point2D(double x_, double y_) : x(x_), y(y_) {}
};


struct Point3D {
  double x, y, z;
  Point3D() {}
  Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};


class IKSolver {
  
public:
  
  double width_;
  double height_;
  double angle_;
  double screen_x_;
  double screen_y_;
  double drag_x_;
  double drag_y_;
  double drag_z_;
  // NOTE: camera matrix is composed of camera extrinsic and intrinsic matrices
  Mat4 cameraMatrix_,
    localToJoint_, // T1*T0
    jointToParent_,   // T3*T4
    parentToWorld_;
  Problem problem_; 
  
  IKSolver() : angle_(0.0), screen_x_(0.0), screen_y_(0.0), drag_x_(0.0), drag_y_(0.0), drag_z_(0.0) {
    buildProblem();
  }
  
  // defined later in this file
  // ugly, but lets me avoid separate class declaration and definition, which would slow me down
  void buildProblem();

  void setLocalToJoint(Mat4 m)  { localToJoint_ = m;  }
  void setJointToParent(Mat4 m) { jointToParent_ = m; }
  void setParentToWorld(Mat4 m) { parentToWorld_ = m; }
  void setCameraMatrix(Mat4 m)  { cameraMatrix_ = m;     }
  
  Point2D projectDragPoint() {
    Eigen::Matrix<double,4,1> drag_point(drag_x_, drag_y_, drag_z_, 1.0);
    Eigen::Matrix<double,2,1> pt(0.0, 0.0);
    // apply projection
    projectPoint(drag_point, pt);
    // copy to output
    Point2D point;
    point.x = pt(0);
    point.y = pt(1);
    return point;
  }

  template<typename T> inline void projectPoint(const Eigen::Matrix<T,4,1> & pt, Eigen::Matrix<T,2,1> & out) {
    Eigen::Matrix<T,4,4> P;
    matToEigen(cameraMatrix_, P);
    // apply projection
    Eigen::Matrix<T,4,1> x = P * pt;
    // from normalized device coords to screen pixel coords
    out(0) = (  x(0) / x(3) + T(1) ) * T(width_  / 2.0);
    out(1) = ( -x(1) / x(3) + T(1) ) * T(height_ / 2.0);
  }

  void setDims(Point2D dims) {
    width_ = dims.x;
    height_ = dims.y;
  }

  double getAngle() const { return angle_; }
  void setAngle(double angle) { angle_ = angle; }
  
  Point2D getScreenPoint() const { return Point2D(screen_x_, screen_y_); }
  void setScreenPoint(Point2D screen_point) { screen_x_ = screen_point.x;
                                              screen_y_ = screen_point.y; }
  
  Point3D getDragPoint() const { return Point3D(drag_x_, drag_y_, drag_z_); }
  void setDragPoint(Point3D drag_point) { drag_x_ = drag_point.x;
                                          drag_y_ = drag_point.y;
                                          drag_z_ = drag_point.z; }

  void stepSolve(int step_limit) {
    // Run the solver!
    Solver::Options options;
    options.max_num_iterations = step_limit;
    options.logging_type = ceres::SILENT;
    options.minimizer_type = ceres::LINE_SEARCH;
    Solver::Summary summary;
    Solve(options, &problem_, &summary);
    cout << summary.FullReport() << "\n";
  }
  
  void timeSolve(double time_limit) {
    // Run the solver!
    Solver::Options options;
    options.max_solver_time_in_seconds = time_limit;
    options.logging_type = ceres::SILENT;
    options.minimizer_type = ceres::LINE_SEARCH;
    Solver::Summary summary;
    Solve(options, &problem_, &summary);
  }
};


struct CostFunctor {

  IKSolver* solver_;

  CostFunctor(IKSolver* solver)
      : solver_(solver) {}

  template <typename T> bool operator()(const T* const angle, T* residual) const {
    // Objective function: residual = Rx - s

    Eigen::Matrix<T,4,1> pt(T(solver_->drag_x_), T(solver_->drag_y_), T(solver_->drag_z_), T(1.0));


    // apply localToJoint_   // T1*T0
    Eigen::Matrix<T,4,4> localToJoint;
    matToEigen(solver_->localToJoint_, localToJoint);
    Eigen::Matrix<T,4,1> pt2 = localToJoint * pt;

    // apply joint rotation
    // the convention in THREE is joint axis = [0, 1, 0]
    T angle_axis[3];
    angle_axis[0] = T(0.0);
    angle_axis[1] = T(*angle) * T(M_PI / 180.0);  // deg to rad
    angle_axis[2] = T(0.0);
    // convert
    T pt2_raw[3];
    pt2_raw[0] = pt2(0) / pt2(3);
    pt2_raw[1] = pt2(1) / pt2(3);
    pt2_raw[2] = pt2(2) / pt2(3);
    // apply rotation
    T pt3_raw[3];
    ceres::AngleAxisRotatePoint<T>(angle_axis, pt2_raw, pt3_raw);

    // apply jointToParent_, parentToWorld_, and camera_matrix_
    Eigen::Matrix<T,4,4> jointToParent, parentToWorld, camera_matrix;
    matToEigen(solver_->jointToParent_, jointToParent);
    matToEigen(solver_->parentToWorld_, parentToWorld);
    
    Eigen::Matrix<T,4,1> pt3(pt3_raw[0], pt3_raw[1], pt3_raw[2], T(1.0));
    Eigen::Matrix<T,4,1> world_pt = parentToWorld * jointToParent * pt3;

    // Camera projection
    Eigen::Matrix<T,2,1> pt_cam(T(0.0), T(0.0));
    solver_->projectPoint(world_pt, pt_cam);

    residual[0] = pt_cam(0) - T(solver_->screen_x_);
    residual[1] = pt_cam(1) - T(solver_->screen_y_);

    return true;
  }

};


void IKSolver::buildProblem() {
    CostFunction* cost_function =
        new AutoDiffCostFunction<CostFunctor, 2, 1>(new CostFunctor(this));
    problem_.AddResidualBlock(cost_function, NULL, &angle_);
}

/*
class RenderingCallback : public IterationCallback {
 public:
  explicit RenderingCallback(double* x)
      : x_(x) {}

  ~RenderingCallback() {}

  CallbackReturnType operator()(const IterationSummary& summary) {
    ostringstream strs;
    strs << "Module.print('x = " << *x_ << "')";
    emscripten_run_script(strs.str().c_str());
    cout << "Iteration " << summary.iteration << endl;
    return SOLVER_CONTINUE;
  }

 private:
  const double* x_;
};
*/

#ifdef EMSCRIPTEN

EMSCRIPTEN_BINDINGS() {
    value_array<Point2D>("Point2D")
        .element(&Point2D::x)
        .element(&Point2D::y)
        ;

    value_array<Point3D>("Point3D")
        .element(&Point3D::x)
        .element(&Point3D::y)
        .element(&Point3D::z)
        ;

    value_array<Mat4>("Mat4")
        .element(&Mat4::a0)
        .element(&Mat4::a1)
        .element(&Mat4::a2)
        .element(&Mat4::a3)
        .element(&Mat4::a4)
        .element(&Mat4::a5)
        .element(&Mat4::a6)
        .element(&Mat4::a7)
        .element(&Mat4::a8)
        .element(&Mat4::a9)
        .element(&Mat4::a10)
        .element(&Mat4::a11)
        .element(&Mat4::a12)
        .element(&Mat4::a13)
        .element(&Mat4::a14)
        .element(&Mat4::a15)
        ;

    class_<IKSolver>("IKSolver")
        .constructor<>()
        .function("stepSolve", &IKSolver::stepSolve)
        .function("timeSolve", &IKSolver::timeSolve)
        .function("setDims", &IKSolver::setDims)
        .function("setCameraMatrix", &IKSolver::setCameraMatrix)
        .function("setLocalToJoint", &IKSolver::setLocalToJoint)
        .function("setJointToParent", &IKSolver::setJointToParent)
        .function("setParentToWorld", &IKSolver::setParentToWorld)
        .function("projectDragPoint", &IKSolver::projectDragPoint)
        .property("screen_point", &IKSolver::getScreenPoint, &IKSolver::setScreenPoint)
        .property("drag_point", &IKSolver::getDragPoint, &IKSolver::setDragPoint)
        .property("angle", &IKSolver::getAngle, &IKSolver::setAngle)
        ;
}

#else

int main(int argc, char** argv) {
  IKSolver solver;
  solver.setDragPoint(Point3D(0.0, 0.5, 0.5));
  solver.setScreenPoint(Point2D(0.5, 0.5));
  solver.stepSolve(10);
  double angle = solver.getAngle();
  cout << "angle = " << angle << "\n";
  double mat[9];
  double angle_axis[3] = {0, 0, angle};
  ceres::AngleAxisToRotationMatrix(angle_axis, mat);
  return 0;
}

#endif
