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


struct Point2D {
  double x;
  double y;
  Point2D() {}
  Point2D(double x_, double y_) : x(x_), y(y_) {}
};


struct CostFunctor {

  double x_, y_, z_;
  double* screen_x_;
  double* screen_y_;

  CostFunctor(double x, double y, double z, double* screen_x, double* screen_y)
      : x_(x), y_(y), z_(z), screen_x_(screen_x), screen_y_(screen_y) {}

  template <typename T> bool operator()(const T* const angle, T* residual) const {
    // Objective function: residual = Rx - s
    
    T pt[3];
    pt[0] = T(x_);
    pt[1] = T(y_);
    pt[2] = T(z_);

    T angle_axis[3];
    angle_axis[0] = T(0.0);
    angle_axis[1] = T(0.0);
    angle_axis[2] = T(*angle);

    T result[3];

    ceres::AngleAxisRotatePoint<T>(angle_axis, pt, result);

    residual[0] = result[0] - T(*screen_x_);
    residual[1] = result[1] - T(*screen_y_);

    return true;
  }

};


class IKSolver {
  
private:
  
  double angle_;
  double screen_x_;
  double screen_y_;
  Problem problem_; 
  
public:
  
  IKSolver() : angle_(0.0), screen_x_(0.0), screen_y_(0.0) {
    buildProblem();
  }
  
  void buildProblem() {
    CostFunction* cost_function =
        new AutoDiffCostFunction<CostFunctor, 2, 1>(new CostFunctor(0.0, 1.0, 0.0, &screen_x_, &screen_y_));
    problem_.AddResidualBlock(cost_function, NULL, &angle_);
  }

  double getAngle() const { return angle_; }
  void setAngle(double angle) { angle_ = angle; }
  
  Point2D getScreenPoint() const { return Point2D(screen_x_, screen_y_); }
  void setScreenPoint(Point2D screen_point) { screen_x_ = screen_point.x;
                                              screen_y_ = screen_point.y; }
  
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

    class_<IKSolver>("IKSolver")
        .constructor<>()
        .function("stepSolve", &IKSolver::stepSolve)
        .function("timeSolve", &IKSolver::timeSolve)
        .property("screen_point", &IKSolver::getScreenPoint, &IKSolver::setScreenPoint)
        .property("angle", &IKSolver::getAngle, &IKSolver::setAngle)
        ;
}

#else

int main(int argc, char** argv) {
  IKSolver solver;
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
