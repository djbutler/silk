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
 
  // Camera information 
  // NOTE: camera matrix is composed of camera extrinsic and intrinsic matrices
  Mat4 cameraMatrix_;
  double width_;
  double height_;
  // 2d drag points
  vector<boost::shared_ptr<Point2D> > targets_2d_;
  vector<boost::shared_ptr<Point3D> > drags_2d_;
  // 3d drag points
  vector<boost::shared_ptr<Point3D> > targets_3d_;
  vector<boost::shared_ptr<Point3D> > drags_3d_;
  // kinematic chain
  vector<boost::shared_ptr<Transform> > transforms;
  int num_joints_;
  int start_transform_index_;
  double robot_state_[7];
  // Ceres problem instance
  Problem problem_; 
  
  IKSolver() : start_transform_index_(0), num_joints_(0) {}
  
  // defined later in this file
  // ugly, but lets me avoid separate class declaration and definition, which would slow me down
  void addTargetPoint2D();
  void addTargetPoint3D();

  int getNumTransforms() { return transforms.size(); }

  double getJointValue(int idx) { return robot_state_[idx]; } 
  void setJointValue(int idx, double val) { robot_state_[idx] = val; } 

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

  // get the pose of a drag point 2D
  Point3D getDragPointPose2D(int idx) {
    typedef double T;
    Eigen::Matrix<T,4,1> v(
        T(drags_2d_[idx]->x),
        T(drags_2d_[idx]->y),
        T(drags_2d_[idx]->z),
        T(1.0)
    );
    Eigen::Matrix<T,4,1> v_tmp;

    for (int i = start_transform_index_; i >= 0; --i) {
        transforms[i]->apply( robot_state_, v, v_tmp );
        v = v_tmp;
    }

    return Point3D(v(0)/v(3), v(1)/v(3), v(2)/v(3));
  }

  // get the pose of a drag point 3D
  Point3D getDragPointPose3D(int idx) {
    typedef double T;
    Eigen::Matrix<T,4,1> v(
        T(drags_3d_[idx]->x),
        T(drags_3d_[idx]->y),
        T(drags_3d_[idx]->z),
        T(1.0)
    );
    Eigen::Matrix<T,4,1> v_tmp;

    for (int i = start_transform_index_; i >= 0; --i) {
        transforms[i]->apply( robot_state_, v, v_tmp );
        v = v_tmp;
    }

    return Point3D(v(0)/v(3), v(1)/v(3), v(2)/v(3));
  }

  // 2D drag points
  void setTargetPoint2D(int idx, Point2D target_point) { *targets_2d_[idx] = target_point; }
  void setDragPoint2D(int idx, Point3D drag_point) { *drags_2d_[idx] = drag_point; }

  // 3D drag points
  void setTargetPoint3D(int idx, Point3D target_point) { *targets_3d_[idx] = target_point; }
  void setDragPoint3D(int idx, Point3D drag_point) { *drags_3d_[idx] = drag_point; }

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


struct CostFunctor2D {

  IKSolver* solver_;
  int index_;

  CostFunctor2D(int idx, IKSolver* solver)
      : solver_(solver), index_(idx) {}

  template <typename T> bool operator()(const T* const robot_state, T* residual) const {
    // Objective function: residual = Rx - s

    Eigen::Matrix<T,4,1> v(
        T(solver_->drags_2d_[index_]->x),
        T(solver_->drags_2d_[index_]->y),
        T(solver_->drags_2d_[index_]->z),
        T(1.0)
    );
    Eigen::Matrix<T,4,1> v_tmp;

    for (int i = solver_->start_transform_index_; i >= 0; --i) {
        solver_->transforms[i]->apply( robot_state, v, v_tmp );
        v = v_tmp;
    }

    // Camera projection
    Eigen::Matrix<T,2,1> pt_cam(T(0.0), T(0.0));
    solver_->projectPoint(v, pt_cam);

    residual[0] = pt_cam(0) - T(solver_->targets_2d_[index_]->x);
    residual[1] = pt_cam(1) - T(solver_->targets_2d_[index_]->y);

    return true;
  }

};

struct CostFunctor3D {

  IKSolver* solver_;
  int index_;

  CostFunctor3D(int idx, IKSolver* solver)
      : solver_(solver), index_(idx) {}

  template <typename T> bool operator()(const T* const robot_state, T* residual) const {
    // Objective function: residual = Rx - s

    Eigen::Matrix<T,4,1> v(
        T(solver_->drags_3d_[index_]->x),
        T(solver_->drags_3d_[index_]->y),
        T(solver_->drags_3d_[index_]->z),
        T(1.0)
    );
    Eigen::Matrix<T,4,1> v_tmp;

    for (int i = solver_->start_transform_index_; i >= 0; --i) {
        solver_->transforms[i]->apply( robot_state, v, v_tmp );
        v = v_tmp;
    }

    residual[0] = v(0) / v(3) - T(solver_->targets_3d_[index_]->x);
    residual[1] = v(1) / v(3) - T(solver_->targets_3d_[index_]->y);
    residual[2] = v(2) / v(3) - T(solver_->targets_3d_[index_]->z);

    return true;
  }

};

void IKSolver::addTargetPoint2D() {
    // add term to objective
    CostFunction* cost_function =
        new AutoDiffCostFunction<CostFunctor2D, 2, 7>(new CostFunctor2D(targets_2d_.size(), this));
    problem_.AddResidualBlock(cost_function, NULL, robot_state_);
    // make room to store the new point
    targets_2d_.push_back(boost::make_shared<Point2D>(0.0, 0.0));
    drags_2d_.push_back(boost::make_shared<Point3D>(0.0, 0.0, 0.0));
}

void IKSolver::addTargetPoint3D() {
    // add term to objective
    CostFunction* cost_function =
        new AutoDiffCostFunction<CostFunctor3D, 3, 7>(new CostFunctor3D(targets_3d_.size(), this));
    problem_.AddResidualBlock(cost_function, NULL, robot_state_);
    // make room to store the new point
    targets_3d_.push_back(boost::make_shared<Point3D>(0.0, 0.0, 0.0));
    drags_3d_.push_back(boost::make_shared<Point3D>(0.0, 0.0, 0.0));
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
        .function("setJointValue", &IKSolver::setJointValue)
        .function("addJointTransform", &IKSolver::addJointTransform)
        .function("addStaticTransform", &IKSolver::addStaticTransform)
        .function("setCameraMatrix", &IKSolver::setCameraMatrix)
        // 2D points
        .function("addTargetPoint2D", &IKSolver::addTargetPoint2D)
        .function("setTargetPoint2D", &IKSolver::setTargetPoint2D)
        .function("setDragPoint2D", &IKSolver::setDragPoint2D)
        // 3D points
        .function("addTargetPoint3D", &IKSolver::addTargetPoint3D)
        .function("setTargetPoint3D", &IKSolver::setTargetPoint3D)
        .function("setDragPoint3D", &IKSolver::setDragPoint3D)
        .function("getDragPointPose3D", &IKSolver::getDragPointPose3D)
        .function("getDragPointPose2D", &IKSolver::getDragPointPose2D)
        // misc
        .function("setStartTransformIndex", &IKSolver::setStartTransformIndex)
        ;
}

#else

int main(int argc, char** argv) {
  IKSolver solver;
  solver.setDragPoint(Point3D(0.0, 0.5, 0.5));
  solver.setScreenPoint(Point2D(0.5, 0.5));
  solver.stepSolve(10);
  double angle = solver.getJointValue(0);
  cout << "angle = " << angle << "\n";
  double mat[9];
  double angle_axis[3] = {0, 0, angle};
  ceres::AngleAxisToRotationMatrix(angle_axis, mat);
  return 0;
}

#endif
