

#include "Costfun.hpp"

#include <ceres/ceres.h>
#include <glog/logging.h>

int main(int argc, char **argv)
{

    google::InitGoogleLogging(argv[0]);

    double x = 0.5;
    const double initial_x = x;

    ceres::Problem problem;
    ceres::CostFunction *cost_fun = new ceres::AutoDiffCostFunction<myCostfun::Costfun, 1, 1>(new myCostfun::Costfun);

    problem.AddResidualBlock(cost_fun, nullptr, &x);

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";

    return 0;
}