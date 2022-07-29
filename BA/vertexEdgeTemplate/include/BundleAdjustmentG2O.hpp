/**
 * 创建一个线性求解器LinearSolver;
 * 创建BlockSolver，并用上面定义的线性求解器初始化;
 * 创建总求解器solver，并从GN/LM/DogLeg 中选一个作为迭代策略，再用上述块求解器BlockSolver初始化;
 * 创建图优化的核心：稀疏优化器（SparseOptimizer）;
 * 定义图的顶点和边，并添加到SparseOptimizer中;
 * 设置优化参数，开始执行优化.
*/

namespace BundleAdjustment
{
    void bundleAdjustmentG2O(int n, int iteration);
}