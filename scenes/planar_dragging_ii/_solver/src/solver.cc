// A simple example of using the Ceres minimizer.
//
#include <stdio.h>
#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "glog/logging.h"

#include "silk/matrices.h"
#include "silk/transforms.h"

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


class IKSolver {
  
public:
  
  double width_;
  double height_;
  double robot_state_[7];
  double target_x_;
  double target_y_;
  double target_z_;
  double drag_x_;
  double drag_y_;
  double drag_z_;
  // NOTE: camera matrix is composed of camera extrinsic and intrinsic matrices
  Mat4 cameraMatrix_;
  vector<boost::shared_ptr<Transform> > transforms;
  Problem problem_; 
  int num_joints_;
  int start_transform_index_;
  
  IKSolver() : start_transform_index_(0), num_joints_(0), target_x_(0.0), target_y_(0.0), target_z_(0.0), drag_x_(0.0), drag_y_(0.0), drag_z_(0.0) {
    buildProblem();
  }
  
  // defined later in this file
  // ugly, but lets me avoid separate class declaration and definition, which would slow me down
  void buildProblem();

  int getNumTransforms() { return transforms.size(); }

  double getJointValue(int idx) { return robot_state_[idx]; } 

  void addJointTransform(double lower_bound, double upper_bound) {
      cout << "Adding joint transform number " << num_joints_ << endl;
      problem_.SetParameterLowerBound(robot_state_, num_joints_, lower_bound);
      problem_.SetParameterUpperBound(robot_state_, num_joints_, upper_bound);
      transforms.push_back(boost::make_shared<JointTransform>(num_joints_++));
  }
  void addStaticTransform(Mat4 m)  {
      transforms.push_back(boost::make_shared<StaticTransform>(m));
  }
  void setCameraMatrix(Mat4 m)  { cameraMatrix_ = m; }

  void setStartTransformIndex(int idx) { start_transform_index_ = idx; }
  
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

  Point3D getTargetPoint() const { return Point3D(target_x_, target_y_, target_z_); }
  void setTargetPoint(Point3D target_point) { target_x_ = target_point.x;
                                              target_y_ = target_point.y;
                                              target_z_ = target_point.z; }
  
  Point3D getDragPoint() const { return Point3D(drag_x_, drag_y_, drag_z_); }
  void setDragPoint(Point3D drag_point) { drag_x_ = drag_point.x;
                                          drag_y_ = drag_point.y;
                                          drag_z_ = drag_point.z; }

  void stepSolve(int step_limit) {
    // Run the solver!
    Solver::Options options;
    options.max_num_iterations = step_limit;
    options.logging_type = ceres::SILENT;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.minimizer_type = ceres::LINE_SEARCH;
    Solver::Summary summary;
    Solve(options, &problem_, &summary);
    cout << summary.FullReport() << "\n";
  }
  
  void timeSolve(double time_limit) {
    // Run the solver!
    Solver::Options options;
    options.max_solver_time_in_seconds = time_limit;
    options.logging_type = ceres::SILENT;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.minimizer_type = ceres::LINE_SEARCH;
    Solver::Summary summary;
    Solve(options, &problem_, &summary);
  }
};


struct CostFunctor {

  IKSolver* solver_;

  CostFunctor(IKSolver* solver)
      : solver_(solver) {}

  template <typename T> bool operator()(const T* const robot_state, T* residual) const {
    // Objective function: residual = Rx - s

    Eigen::Matrix<T,4,1> v(T(solver_->drag_x_), T(solver_->drag_y_), T(solver_->drag_z_), T(1.0));
    Eigen::Matrix<T,4,1> v_tmp;

    //T* state;
    //T tmp = T(*angle);
    //state = &tmp;

    for (int i = solver_->start_transform_index_; i >= 0; --i) {
        //solver_->transforms[i]->apply( state, v, v_tmp );
        solver_->transforms[i]->apply( robot_state, v, v_tmp );
        v = v_tmp;
    }

    // Camera projection
    /*
    Eigen::Matrix<T,2,1> pt_cam(T(0.0), T(0.0));
    solver_->projectPoint(v, pt_cam);
    */

    residual[0] = v(0)/v(3) - T(solver_->target_x_);
    residual[1] = v(1)/v(3) - T(solver_->target_y_);
    residual[2] = v(2)/v(3) - T(solver_->target_z_);

    return true;
  }

};


void IKSolver::buildProblem() {
    CostFunction* cost_function =
        new AutoDiffCostFunction<CostFunctor, 3, 7>(new CostFunctor(this));
    problem_.AddResidualBlock(cost_function, NULL, robot_state_);
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
        .function("getNumTransforms", &IKSolver::getNumTransforms)
        .function("getJointValue", &IKSolver::getJointValue)
        .function("addJointTransform", &IKSolver::addJointTransform)
        .function("addStaticTransform", &IKSolver::addStaticTransform)
        .function("setCameraMatrix", &IKSolver::setCameraMatrix)
        .function("projectDragPoint", &IKSolver::projectDragPoint)
        .property("target_point", &IKSolver::getTargetPoint, &IKSolver::setTargetPoint)
        .property("drag_point", &IKSolver::getDragPoint, &IKSolver::setDragPoint)
        .function("setStartTransformIndex", &IKSolver::setStartTransformIndex)
        ;
}

#else

int main(int argc, char** argv) {
  IKSolver solver;
  solver.setDragPoint(Point3D(0.0, 0.5, 0.5));
  solver.setTargetPoint(Point3D(0.5, 0.5, 0.5));
  solver.stepSolve(10);
  double angle = solver.getJointValue(0);
  cout << "angle = " << angle << "\n";
  double mat[9];
  double angle_axis[3] = {0, 0, angle};
  ceres::AngleAxisToRotationMatrix(angle_axis, mat);
  return 0;
}

#endif
