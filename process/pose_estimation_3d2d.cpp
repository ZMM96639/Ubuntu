#include <iostream>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <chrono>

using namespace std;
using namespace cv;

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

// BA by g2o
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

void bundleAdjustmentG2O(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const Mat &K,
        Sophus::SE3d &pose
);

// BA by gauss-newton
void bundleAdjustmentGaussNewton(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const Mat &K,
        Sophus::SE3d &pose
);

int main(int argc, char **argv) {
    vector<Point3f> pts_3d, camera;
    vector<Point2f> pts_2d;
    int i = 0, k = 0, l = 0;
    int Arsize = 208;
    ifstream infile_1, infile_2, infile_3;
    string s, m, h;
    vector<string> v1, v2, v3;
    double data_1[Arsize][3], data_2[Arsize][2], data_3[3][3];

    infile_1.open(argv[1]);
    if (!infile_1.is_open()) {
        cout << "please check out the path of file" << endl;
    }

    //3D
    while (getline(infile_1, s)) {
        v1.push_back(s);
    }
    cout << "3D_coordinary:\n";
    for (; i < Arsize; i++) {
        string tmp;
        tmp = accumulate(v1[i].begin(), v1[i].end(), tmp);
        char *s_input = (char *) tmp.c_str();
        const char *split = ",";
        char *p = strtok(s_input, split);
        double a;
        int j = 0;
        while (p != NULL) {
            sscanf(p, "%lf", &a);
            p = strtok(NULL, split);
            data_1[i][j] = a;
            j++;
        }
        pts_3d.push_back(Point3f(data_1[i][0], data_1[i][1], data_1[i][2]));
    }
    cout << pts_3d << endl;

    //pixel
    infile_2.open(argv[2]);
    if (!infile_2.is_open()) {
        cout << "please check out the path of file" << endl;
    }
    while (getline(infile_2, m)) {
        v2.push_back(m);
    }
    cout << "\npixel_coordinary:\n";
    for (; k < Arsize; k++) {
        string tmp;
        tmp = accumulate(v2[k].begin(), v2[k].end(), tmp);
        char *s_input = (char *) tmp.c_str();
        const char *split = ",";
        char *p = strtok(s_input, split);
        double a;
        int j = 0;
        while (p != NULL) {
            sscanf(p, "%lf", &a);
            p = strtok(NULL, split);
            data_2[k][j] = a;
            j++;
        }
        pts_2d.push_back(Point2f(data_2[k][0], data_2[k][1]));
    }
    cout << pts_2d << endl;

    infile_3.open(argv[3]);
    if (!infile_3.is_open()) {
        cout << "please check out the path of file" << endl;
    }

    //im
    while (getline(infile_3, h)) {
        v3.push_back(h);
    }

    for (; l < 3; l++) {
        string tmp;
        tmp = accumulate(v3[l].begin(), v3[l].end(), tmp);
        char *s_input = (char *) tmp.c_str();
        const char *split = ",";
        char *p = strtok(s_input, split);
        double a;
        int j = 0;
        while (p != NULL) {
            sscanf(p, "%lf", &a);
            p = strtok(NULL, split);
            data_3[l][j] = a;
            j++;
        }
//        camera.push_back(Point3f(data_3[l][0], data_3[l][1], data_3[l][2]));
    }
// 建立3D点
//    Mat K = (Mat_<double>(3, 3)
//            << 2864.389051, 0.132770, 1306.081919, 0, 2864.682667, 1042.003042, 0, 0, 1);
    Mat K = (Mat_<double>(3, 3)
            << data_3[0][0], data_3[0][1], data_3[0][2], data_3[1][0], data_3[1][1], data_3[1][2], 0, 0, 1);
    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for (size_t n = 0; n < pts_3d.size(); ++n) {
        pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[n].x, pts_3d[n].y, pts_3d[n].z)
        );
        pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[n].x, pts_2d[n].y)
        );
    }
    cout << "calling bundle adjustment by gauss newton" << endl;
    Sophus::SE3d pose_gn;
    bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
    cout << "calling bundle adjustment by g2o" << endl;
    Sophus::SE3d pose_g2o;
    bundleAdjustmentG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);

    return 0;
}

//Point2d pixel2cam(const Point2d &p, const Mat &K) {
//    return Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
//                   (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
//}

void bundleAdjustmentGaussNewton(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const Mat &K,
        Sophus::SE3d &pose) {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    const int iterations = 10;
    double cost = 0, lastCost = 0;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    for (int iter = 0; iter < iterations; iter++) {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
        cost = 0;
        // compute cost
        for (int num = 0; num < points_3d.size(); num++) {
            Eigen::Vector3d pc = pose * points_3d[num];//coordinate:world to camera
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
            Eigen::Vector2d e = points_2d[num] - proj;
            cost += e.squaredNorm();
            Eigen::Matrix<double, 2, 6> J;
            J << -fx * inv_z,
                    0,
                    fx * pc[0] * inv_z2,
                    fx * pc[0] * pc[1] * inv_z2,
                    -fx - fx * pc[0] * pc[0] * inv_z2,
                    fx * pc[1] * inv_z,
                    0,
                    -fy * inv_z,
                    fy * pc[1] * inv_z2,
                    fy + fy * pc[1] * pc[1] * inv_z2,
                    -fy * pc[0] * pc[1] * inv_z2,
                    -fy * pc[0] * inv_z;
            H += J.transpose() * J;
            b += -J.transpose() * e;
        }
        Vector6d dx;
        dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }
        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }
        // update your estimation
        pose = Sophus::SE3d::exp(dx) * pose;
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << endl;
        if (dx.norm() < 1e-6) {
            // converge
            break;
        }
    }
    cout << "pose by g-n: \n" << pose.matrix() << endl;
}
/// vertex and edges used in g2o ba
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
        _estimate = Sophus::SE3d();
    }

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override {
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    virtual bool read(istream &in) override {}

    virtual bool write(ostream &out) const override {}
};

class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}

    virtual void computeError() override {
        const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }

    virtual void linearizeOplus() override {
        const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pos_cam = T * _pos3d;
        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double cx = _K(0, 2);
        double cy = _K(1, 2);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Z2 = Z * Z;
        _jacobianOplusXi
                << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
                0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
    }

    virtual bool read(istream &in) override {}

    virtual bool write(ostream &out) const override {}

private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
};

void bundleAdjustmentG2O(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const Mat &K,
        Sophus::SE3d &pose) {
    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;  // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(true);       // 打开调试输出
    // vertex
    VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertex_pose);
    // K
    Eigen::Matrix3d K_eigen;
    K_eigen <<
            K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
            K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
            K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);
    // edges
    int index = 8;
    for (size_t i = 0; i < points_2d.size(); ++i) {
        auto p2d = points_2d[i];
        auto p3d = points_3d[i];
        EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
        edge->setId(index);
        edge->setVertex(0, vertex_pose);
        edge->setMeasurement(p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
    cout << "pose estimated by g2o =\n" << vertex_pose->estimate().matrix() << endl;
    pose = vertex_pose->estimate();
}
