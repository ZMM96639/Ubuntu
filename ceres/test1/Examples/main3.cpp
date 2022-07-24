

#include "Costfun.hpp"
#include "data.h"

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

using namespace myCostfun;
using namespace Data;

// Data generated using the following octave code.
//   randn('seed', 23497);
//   m = 0.3;
//   c = 0.1;
//   x=[0:0.075:5];
//   y = exp(m * x + c);
//   noise = randn(size(x)) * 0.2;
//   y_observed = y + noise;
//   data = [x', y_observed'];
const int kNumObservations = 67;
// clang-format off


int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    double m = 0.0;
    double c = 0.0;
    Problem problem;
    for (int i = 0; i < kNumObservations; ++i)
    {
        problem.AddResidualBlock(
            new AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
                new ExponentialResidual(data[2 * i], data[2 * i + 1])),nullptr,&m,&c);
    }
    Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    std::cout << "Final   m: " << m << " c: " << c << "\n";
    return 0;
}