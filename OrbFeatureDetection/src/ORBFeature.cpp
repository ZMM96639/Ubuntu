#include "ORB/ORBFeature.hpp"
#include <memory>
#include <string>

namespace ORB
{
    ORBFeature::ORBFeature(const std::string &image_path, const std::string &config_path)
    {
        image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        camera_ptr = std::make_shared<Parameter>(config_path); // 相机内参加载
        setPara();
    }

    // TODO 利用OpenCV函数进行ORB特征点的提取
    void ORBFeature::ExtractORB()
    {
        if (image.empty())
            return;

        // 判断图像的格式是否正确，要求是单通道灰度值
        assert(image.type() == CV_8UC1);

        auto orbPtr = cv::ORB::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::FAST_SCORE, 31, 20);

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        orbPtr->detectAndCompute(image, Mat(), keypoints, descriptors);

        LOG(INFO) << "/**************************/" << std::endl;
        LOG(INFO) << "OpenCV提取的结果:" << std::endl;
        LOG(INFO) << "Keypoints size " << keypoints.size();
        LOG(INFO) << "descriptor size " << descriptors.rows;
        LOG(INFO) << "/**************************/" << std::endl;

        cv::Mat outImage;
        cv::drawKeypoints(image, keypoints, outImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        cv::imshow("这是OpenCV提取的特征点", outImage);
    }

    // TODO 使用ORB-SLAM2的方法进行ORB特征点的提取
    void ORBFeature::ExtractORB(std::vector<cv::KeyPoint> &_keypoints, cv::Mat &_descriptors)
    {
        // Step 1 检查图像有效性。如果图像为空，那么就直接返回
        if (image.empty())
            return;

        // 获取图像的大小
        // Mat image = _image.getMat();

        // 判断图像的格式是否正确，要求是单通道灰度值
        assert(image.type() == CV_8UC1);

        // Pre-compute the scale pyramid
        // Step 2 构建图像金字塔
        ComputePyramid();

        // Step 3 计算图像的特征点，并且将特征点进行均匀化。均匀的特征点可以提高位姿计算精度
        // 存储所有的特征点，注意此处为二维的vector，第一维存储的是金字塔的层数，第二维存储的是那一层金字塔图像里提取的所有特征点
        std::vector<std::vector<cv::KeyPoint>> allKeypoints;

        // 使用四叉树的方式计算每层图像的特征点并进行分配
        ComputeKeyPointsQuadtree(allKeypoints);

        // Step 4 拷贝图像描述子到新的矩阵descriptors
        Mat descriptors;

        // 统计整个图像金字塔中的特征点
        int nkeypoints = 0;

        int nlevels = camera_ptr->nLevels;

        // 开始遍历每层图像金字塔，并且累加每层的特征点个数
        for (int level = 0; level < nlevels; ++level)
            nkeypoints += (int)allKeypoints[level].size();

        // 如果本图像金字塔中没有任何的特征点
        if (nkeypoints == 0)
            // 通过调用cv::mat类的.realse方法，强制清空矩阵的引用计数，这样就可以强制释放矩阵的数据了
            // 参考[https://blog.csdn.net/giantchen547792075/article/details/9107877]
            _descriptors.release();
        else
        {
            // 如果图像金字塔中有特征点，那么就创建这个存储描述子的矩阵，注意这个矩阵是存储整个图像金字塔中特征点的描述子的
            _descriptors.create(nkeypoints, // 矩阵的行数，对应为特征点的总个数
                                32,         // 矩阵的列数，对应为使用32*8=256位描述子
                                CV_8U);     // 矩阵元素的格式

            // 获取这个描述子的矩阵信息
            // ?为什么不是直接在参数_descriptors上对矩阵内容进行修改，而是重新新建了一个变量，复制矩阵后，在这个新建变量的基础上进行修改？
            descriptors = _descriptors;
        }

        // 清空用作返回特征点提取结果的vector容器
        _keypoints.clear();

        // 并预分配正确大小的空间
        _keypoints.reserve(nkeypoints);

        // 因为遍历是一层一层进行的，但是描述子那个矩阵是存储整个图像金字塔中特征点的描述子，所以在这里设置了Offset变量来保存“寻址”时的偏移量，
        // 辅助进行在总描述子mat中的定位
        int offset = 0;
        
        // 开始遍历每一层图像
        for (int level = 0; level < nlevels; ++level)
        {
            // 获取在allKeypoints中当前层特征点容器的句柄
            std::vector<cv::KeyPoint> &keypoints = allKeypoints[level];
            // 本层的特征点数
            int nkeypointsLevel = (int)keypoints.size();

            // 如果特征点数目为0，跳出本次循环，继续下一层金字塔
            if (nkeypointsLevel == 0)
                continue;

            // preprocess the resized image
            //  Step 5 对图像进行高斯模糊
            // 深拷贝当前金字塔所在层级的图像
            Mat workingMat = mvImagePyramid[level].clone();

            // 注意：提取特征点的时候，使用的是清晰的原图像；这里计算描述子的时候，为了避免图像噪声的影响，使用了高斯模糊
            GaussianBlur(workingMat,              // 源图像
                         workingMat,              // 输出图像
                         cv::Size(7, 7),          // 高斯滤波器kernel大小，必须为正的奇数
                         2,                       // 高斯滤波在x方向的标准差
                         2,                       // 高斯滤波在y方向的标准差
                         cv::BORDER_REFLECT_101); // 边缘拓展点插值类型

            // Compute the descriptors 计算描述子
            // desc存储当前图层的描述子
            Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
            
            // Step 6 计算高斯模糊后图像的描述子
            computeDescriptors(workingMat, // 高斯模糊之后的图层图像
                               keypoints,  // 当前图层中的特征点集合
                               desc,       // 存储计算之后的描述子
                               pattern);   // 随机采样模板

            // 更新偏移量的值
            offset += nkeypointsLevel;

            // Scale keypoint coordinates
            // Step 6 对非第0层图像中的特征点的坐标恢复到第0层图像（原图像）的坐标系下
            // ? 得到所有层特征点在第0层里的坐标放到_keypoints里面
            // 对于第0层的图像特征点，他们的坐标就不需要再进行恢复了
            if (level != 0)
            {
                // 获取当前图层上的缩放系数
                float scale = mvScaleFactor[level];
                // 遍历本层所有的特征点
                for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
                                                         keypointEnd = keypoints.end();
                     keypoint != keypointEnd; ++keypoint)
                    // 特征点本身直接乘缩放倍数就可以了
                    keypoint->pt *= scale;
            }

            // And add the keypoints to the output
            // 将keypoints中内容插入到_keypoints 的末尾
            // keypoint其实是对allkeypoints中每层图像中特征点的引用，这样allkeypoints中的所有特征点在这里被转存到输出的_keypoints
            _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
        }
    }

    void ORBFeature::UndistortImage()
    {
        int rows = image.rows;
        int cols = image.cols;
        cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);

        for (int v = 0; v < rows; v++)
        {
            for (int u = 0; u < cols; u++)
            {
                double x = (u - camera_ptr->cx) / camera_ptr->fx;
                double y = (v - camera_ptr->cy) / camera_ptr->fy;

                double r = sqrt(x * x + y * y);
                double r2 = r * r;
                double r4 = r2 * r2;

                double x_dis = x * (1 + camera_ptr->k1 * r2 + camera_ptr->k2 * r4) + 2 * camera_ptr->p1 * x * y + camera_ptr->p2 * (r2 + 2 * x * x);
                double y_dis = y * (1 + camera_ptr->k1 * r2 + camera_ptr->k2 * r4) + camera_ptr->p1 * (r2 + 2 * y * y) + 2 * camera_ptr->p2 * x * y;

                double u_dis = camera_ptr->fx * x_dis + camera_ptr->cx;
                double v_dis = camera_ptr->fy * y_dis + camera_ptr->cy;

                if (u_dis >= 0 && v_dis >= 0 && u_dis < cols && v_dis < rows)
                {
                    image_undistort.at<uchar>(v, u) = image.at<uchar>((int)v_dis, (int)u_dis);
                }
                else
                {
                    image_undistort.at<uchar>(v, u) = 0;
                }
            }
        }

        cv::imshow("distored", image);
        cv::imshow("undistorted", image_undistort);
        cv::waitKey();
    }

    int ORBFeature::HALF_PATCH_SIZE = 15;
    int ORBFeature::EDGE_THRESHOLD = 19;
    int ORBFeature::PATCH_SIZE = 31;

    static int bit_pattern_31_[256 * 4] =
    {
        #include "ORB/pattern.inc"
    };

    void ORBFeature::setPara()
    {
        int nlevels = camera_ptr->nLevels;
        double scaleFactor = camera_ptr->scalseFactor;
        int nfeatures = camera_ptr->nFeatures;

        mvScaleFactor.resize(nlevels);
        mvLevelSigma2.resize(nlevels);
        mvScaleFactor[0] = 1.0f;
        mvLevelSigma2[0] = 1.0f;
        for (int i = 1; i < nlevels; i++)
        {
            mvScaleFactor[i] = mvScaleFactor[i - 1] * scaleFactor;
            mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i];
        }

        mvInvScaleFactor.resize(nlevels);
        mvInvLevelSigma2.resize(nlevels);
        for (int i = 0; i < nlevels; i++)
        {
            mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i];
            mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
        }

        mvImagePyramid.resize(nlevels);
        mnFeaturesPerLevel.resize(nlevels);
        float factor = 1.0f / scaleFactor;
        float nDesiredFeaturesPerScale = nfeatures * (1 - factor) / (1 - (float)pow((double)factor, (double)nlevels));
        int sumFeatures = 0;
        for (int level = 0; level < nlevels - 1; level++)
        {
            mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
            // 累计
            sumFeatures += mnFeaturesPerLevel[level];
            // 乘系数
            nDesiredFeaturesPerScale *= factor;
        }

        mnFeaturesPerLevel[nlevels - 1] = std::max(nfeatures - sumFeatures, 0);
        const int npoints = 512;

        const cv::Point *pattern0 = (const cv::Point *)bit_pattern_31_;

        std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

        umax.resize(HALF_PATCH_SIZE + 1);

        int v,  // 循环辅助变量
            v0, // 辅助变量
            vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);

        int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);

        const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;

        // 利用圆的方程计算每行像素的u坐标边界（max）
        for (v = 0; v <= vmax; ++v)
            umax[v] = cvRound(sqrt(hp2 - v * v));

        // 这里其实是使用了对称的方式计算上四分之一的圆周上的umax，目的也是为了保持严格的对称 关于u = v 直线
        for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
        {
            while (umax[v0] == umax[v0 + 1])
                ++v0;
            umax[v] = v0;
            ++v0;
        }
    }

    void ORBFeature::ComputePyramid()
    {
        int nlevels = camera_ptr->nLevels;

        // cv::imshow("origImage", image);

        for (int level = 0; level < nlevels; ++level)
        {
            float scale = mvInvScaleFactor[level];
            cv::Size sz(cvRound((float)image.cols * scale), cvRound((float)image.rows * scale));

            // 全尺寸图像。包括无效图像区域的大小。将图像进行“补边”，EDGE_THRESHOLD区域外的图像不进行FAST角点检测
            cv::Size wholeSize(sz.width + EDGE_THRESHOLD * 2, sz.height + EDGE_THRESHOLD * 2);
            cv::Mat temp(wholeSize, image.type()), masktemp;

            // Rect(左上角x坐标，左上角y坐标，矩形的宽，矩形的高)
            mvImagePyramid[level] = temp(cv::Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));
            if (level != 0)
            {
                // 将上一层金字塔图像根据设定sz缩放到当前层级
                //  resize(mvImagePyramid[level-1],	//输入图像
                //      mvImagePyramid[level], 	//输出图像
                //      sz, 						//输出图像的尺寸
                //      0, 						//水平方向上的缩放系数，留0表示自动计算
                //      0,  						//垂直方向上的缩放系数，留0表示自动计算
                //      cv::INTER_LINEAR);		//图像缩放的差值算法类型，这里的是线性插值算法

                //!  原代码mvImagePyramid 并未扩充，上面resize应该改为如下
                cv::resize(image,                                                 // 输入图像
                           mvImagePyramid[level],                                 // 输出图像
                           sz,                                                    // 输出图像的尺寸
                           0,                                                     // 水平方向上的缩放系数，留0表示自动计算
                           0,                                                     // 垂直方向上的缩放系数，留0表示自动计算
                           cv::INTER_LINEAR);                                     // 图像缩放的差值算法类型，这里的是线性插值算法

                cv::copyMakeBorder(mvImagePyramid[level],                         // 源图像
                                   temp,                                          // 目标图像（此时其实就已经有大了一圈的尺寸了）
                                   EDGE_THRESHOLD, EDGE_THRESHOLD,                // top & bottom 需要扩展的border大小
                                   EDGE_THRESHOLD, EDGE_THRESHOLD,                // left & right 需要扩展的border大小
                                   cv::BORDER_REFLECT_101 + cv::BORDER_ISOLATED); // 扩充方式，opencv给出的解释：
            }
            else
            {
                cv::copyMakeBorder(image, // 这里是原图像
                                   temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                                   cv::BORDER_REFLECT_101);
            }
            mvImagePyramid[level] = temp;

            // cv::imshow("mvImagePyramid["+ std::to_string(level)+"]", mvImagePyramid[level]);
            // cv::waitKey(0);
        }
    }

    void ORBFeature::ComputeKeyPointsQuadtree(std::vector<std::vector<cv::KeyPoint>> &allKeypoints)
    {
        int nlevels = camera_ptr->nLevels;
        int nfeatures = camera_ptr->nFeatures;
        int iniThFAST = camera_ptr->iniThFAST;
        int minThFAST = camera_ptr->minThFAST;

        allKeypoints.resize(nlevels);

        // 图像cell的尺寸，是个正方形，可以理解为边长in像素坐标
        const float W = 30;

        for (int level = 0; level < nlevels; ++level)
        {
            // 计算这层图像的坐标边界
            // NOTICE 注意这里是坐标边界，EDGE_THRESHOLD指的应该是可以提取特征点的有效图像边界，后面会一直使用“有效图像边界“这个自创名词
            const int minBorderX = EDGE_THRESHOLD - 3; // 这里的3是因为在计算FAST特征点的时候，需要建立一个半径为3的圆
            const int minBorderY = minBorderX;         // minY的计算就可以直接拷贝上面的计算结果了
            const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD + 3;
            const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD + 3;

            // 存储需要进行平均分配的特征点
            std::vector<cv::KeyPoint> vToDistributeKeys;

            // 一般地都是过量采集，所以这里预分配的空间大小是nfeatures*10
            vToDistributeKeys.reserve(nfeatures * 10);

            // 计算进行特征点提取的图像区域尺寸
            const float width = (maxBorderX - minBorderX);
            const float height = (maxBorderY - minBorderY);

            // 计算网格在当前层的图像有的行数和列数
            const int nCols = width / W;
            const int nRows = height / W;

            // 计算每个图像网格所占的像素行数和列数
            const int wCell = ceil(width / nCols);
            const int hCell = ceil(height / nRows);

            // 开始遍历图像网格，还是以行开始遍历的
            for (int i = 0; i < nRows; i++)
            {
                // 计算当前网格初始行坐标
                const float iniY = minBorderY + i * hCell;
                
                // 计算当前网格最大的行坐标，这里的+6=+3+3，即考虑到了多出来3是为了cell边界像素进行FAST特征点提取用
                // 前面的EDGE_THRESHOLD指的应该是提取后的特征点所在的边界，所以minBorderY是考虑了计算半径时候的图像边界
                // 目测一个图像网格的大小是25*25啊
                float maxY = iniY + hCell + 6;

                // 如果初始的行坐标就已经超过了有效的图像边界了，这里的“有效图像”是指原始的、可以提取FAST特征点的图像区域
                if (iniY >= maxBorderY - 3)
                    // 那么就跳过这一行
                    continue;

                // 如果图像的大小导致不能够正好划分出来整齐的图像网格，那么就要委屈最后一行了
                if (maxY > maxBorderY)
                    maxY = maxBorderY;

                // 开始列的遍历
                for (int j = 0; j < nCols; j++)
                {
                    // 计算初始的列坐标
                    const float iniX = minBorderX + j * wCell;

                    // 计算这列网格的最大列坐标，+6的含义和前面相同
                    float maxX = iniX + wCell + 6;

                    // 判断坐标是否在图像中
                    // 如果初始的列坐标就已经超过了有效的图像边界了，这里的“有效图像”是指原始的、可以提取FAST特征点的图像区域。
                    // 并且应该同前面行坐标的边界对应，都为-3
                    //! BUG  正确应该是maxBorderX-3
                    if (iniX >= maxBorderX - 6)
                        continue;

                    // 如果最大坐标越界那么委屈一下
                    if (maxX > maxBorderX)
                        maxX = maxBorderX;

                    // FAST提取兴趣点, 自适应阈值
                    // 这个向量存储这个cell中的特征点
                    std::vector<cv::KeyPoint> vKeysCell;

                    // 调用opencv的库函数来检测FAST角点
                    cv::FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX), // 待检测的图像，这里就是当前遍历到的图像块
                             vKeysCell,                                                       // 存储角点位置的容器
                             iniThFAST,                                                       // 检测阈值
                             true);                                                           // 使能非极大值抑制

                    // 如果这个图像块中使用默认的FAST检测阈值没有能够检测到角点
                    if (vKeysCell.empty())
                    {
                        // 那么就使用更低的阈值来进行重新检测
                        FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX), // 待检测的图像
                             vKeysCell,                                                       // 存储角点位置的容器
                             minThFAST,                                                       // 更低的检测阈值
                             true);                                                           // 使能非极大值抑制
                    }

                    // 当图像cell中检测到FAST角点的时候执行下面的语句
                    if (!vKeysCell.empty())
                    {
                        // 遍历其中的所有FAST角点
                        for (std::vector<cv::KeyPoint>::iterator vit = vKeysCell.begin(); vit != vKeysCell.end(); vit++)
                        {
                            // NOTICE 到目前为止，这些角点的坐标都是基于图像cell的，现在我们要先将其恢复到当前的【坐标边界】下的坐标
                            // 这样做是因为在下面使用八叉树法整理特征点的时候将会使用得到这个坐标
                            // 在后面将会被继续转换成为在当前图层的扩充图像坐标系下的坐标
                            (*vit).pt.x += j * wCell;
                            (*vit).pt.y += i * hCell;

                            // 然后将其加入到”等待被分配“的特征点容器中
                            vToDistributeKeys.push_back(*vit);
                            
                        } // 遍历图像cell中的所有的提取出来的FAST角点，并且恢复其在整个金字塔当前层图像下的坐标
                    }     // 当图像cell中检测到FAST角点的时候执行下面的语句
                }         // 开始遍历图像cell的列
            }             // 开始遍历图像cell的行

            // 声明一个对当前图层的特征点的容器的引用
            std::vector<cv::KeyPoint> &keypoints = allKeypoints[level];

            // 并且调整其大小为欲提取出来的特征点个数（当然这里也是扩大了的，因为不可能所有的特征点都是在这一个图层中提取出来的）
            keypoints.reserve(nfeatures);

            // 根据 mnFeatuvector<KeyPoint> &keypoints = allKeypoints[level]; resPerLevel,即该层的兴趣点数,对特征点进行剔除
            // 返回值是一个保存有特征点的vector容器，含有剔除后的保留下来的特征点
            // 得到的特征点的坐标，依旧是在当前图层下来讲的
            keypoints = DistributeQuadtree(vToDistributeKeys,      // 当前图层提取出来的特征点，也即是等待剔除的特征点
                                                                   // NOTICE 注意此时特征点所使用的坐标都是在“半径扩充图像”下的
                                           minBorderX, maxBorderX, // 当前图层图像的边界，而这里的坐标却都是在“边缘扩充图像”下的
                                           minBorderY, maxBorderY,
                                           mnFeaturesPerLevel[level], // 希望保留下来的当前层图像的特征点个数
                                           level);                    // 当前层图像所在的图层

            // PATCH_SIZE是对于底层的初始图像来说的，现在要根据当前图层的尺度缩放倍数进行缩放得到缩放后的PATCH大小 和特征点的方向计算有关
            const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

            // Add border to coordinates and scale information
            // 获取剔除过程后保留下来的特征点数目
            const int nkps = keypoints.size();

            // 然后开始遍历这些特征点，恢复其在当前图层图像坐标系下的坐标
            for (int i = 0; i < nkps; i++)
            {
                // 对每一个保留下来的特征点，恢复到相对于当前图层“边缘扩充图像下”的坐标系的坐标
                keypoints[i].pt.x += minBorderX;
                keypoints[i].pt.y += minBorderY;

                // 记录特征点来源的图像金字塔图层
                keypoints[i].octave = level;

                // 记录计算方向的patch，缩放后对应的大小， 又被称作为特征点半径
                keypoints[i].size = scaledPatchSize;
            }
        }

        for (int level = 0; level < nlevels; ++level)
            computeOrientation(mvImagePyramid[level], // 对应的图层的图像
                               allKeypoints[level],   // 这个图层中提取并保留下来的特征点容器
                               umax);                 // 以及PATCH的横坐标边界
    }

    std::vector<cv::KeyPoint> ORBFeature::DistributeQuadtree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                                                             const int &maxX, const int &minY, const int &maxY, const int &N, const int &level)
    {
        // Compute how many initial nodes
        // Step 1 根据宽高比确定初始节点数目
        // 计算应该生成的初始节点个数，根节点的数量nIni是根据边界的宽高比值确定的，一般是1或者2
        // ! bug: 如果宽高比小于0.5，nIni = 0, 后面hx会报错
        const int nIni = round(static_cast<float>(maxX - minX) / (maxY - minY));

        // 一个初始的节点的x方向有多少个像素
        const float hX = static_cast<float>(maxX - minX) / nIni;

        // 存储有提取器节点的链表
        std::list<ExtractorNode> lNodes;

        // 存储初始提取器节点指针的vector
        std::vector<ExtractorNode *> vpIniNodes;

        // 重新设置其大小
        vpIniNodes.resize(nIni);

        // Step 2 生成初始提取器节点
        for (int i = 0; i < nIni; i++)
        {
            // 生成一个提取器节点
            ExtractorNode ni;

            // 设置提取器节点的图像边界
            // 注意这里和提取FAST角点区域相同，都是“半径扩充图像”，特征点坐标从0 开始
            ni.UL = cv::Point2i(hX * static_cast<float>(i), 0);     // UpLeft
            ni.UR = cv::Point2i(hX * static_cast<float>(i + 1), 0); // UpRight
            ni.BL = cv::Point2i(ni.UL.x, maxY - minY);              // BottomLeft
            ni.BR = cv::Point2i(ni.UR.x, maxY - minY);              // BottomRight

            // 重设vkeys大小
            ni.vKeys.reserve(vToDistributeKeys.size());

            // 将刚才生成的提取节点添加到链表中
            // 虽然这里的ni是局部变量，但是由于这里的push_back()是拷贝参数的内容到一个新的对象中然后再添加到列表中
            // 所以当本函数退出之后这里的内存不会成为“野指针”
            lNodes.push_back(ni);

            // 存储这个初始的提取器节点句柄
            vpIniNodes[i] = &lNodes.back();
        }

        // Associate points to childs
        //  Step 3 将特征点分配到子提取器节点中
        for (size_t i = 0; i < vToDistributeKeys.size(); i++)
        {
            // 获取这个特征点对象
            const cv::KeyPoint &kp = vToDistributeKeys[i];

            // 按特征点的横轴位置，分配给属于那个图像区域的提取器节点（最初的提取器节点）
            vpIniNodes[kp.pt.x / hX]->vKeys.push_back(kp);
        }

        // Step 4 遍历此提取器节点列表，标记那些不可再分裂的节点，删除那些没有分配到特征点的节点
        // ? 这个步骤是必要的吗？感觉可以省略，通过判断nIni个数和vKeys.size() 就可以吧
        std::list<ExtractorNode>::iterator lit = lNodes.begin();

        while (lit != lNodes.end())
        {
            // 如果初始的提取器节点所分配到的特征点个数为1
            if (lit->vKeys.size() == 1)
            {
                // 那么就标志位置位，表示此节点不可再分
                lit->bNoMore = true;

                // 更新迭代器
                lit++;
            }
            
            // 如果一个提取器节点没有被分配到特征点，那么就从列表中直接删除它
            else if (lit->vKeys.empty())

                // 注意，由于是直接删除了它，所以这里的迭代器没有必要更新；否则反而会造成跳过元素的情况
                lit = lNodes.erase(lit);
            else
                // 如果上面的这些情况和当前的特征点提取器节点无关，那么就只是更新迭代器
                lit++;
        }

        // 结束标志位清空
        bool bFinish = false;

        // 记录迭代次数，只是记录，并未起到作用
        int iteration = 0;

        // 声明一个vector用于存储节点的vSize和句柄对
        // 这个变量记录了在一次分裂循环中，那些可以再继续进行分裂的节点中包含的特征点数目和其句柄
        std::vector<std::pair<int, ExtractorNode *>> vSizeAndPointerToNode;

        // 调整大小，这里的意思是一个初始化节点将“分裂”成为四个
        vSizeAndPointerToNode.reserve(lNodes.size() * 4);

        // Step 5 利用四叉树方法对图像进行划分区域，均匀分配特征点
        while (!bFinish)
        {
            // 更新迭代次数计数器，只是记录，并未起到作用
            iteration++;

            // 保存当前节点个数，prev在这里理解为“保留”比较好
            int prevSize = lNodes.size();

            // 重新定位迭代器指向列表头部
            lit = lNodes.begin();

            // 需要展开的节点计数，这个一直保持累计，不清零
            int nToExpand = 0;

            // 因为是在循环中，前面的循环体中可能污染了这个变量，所以清空
            // 这个变量也只是统计了某一个循环中的点
            // 这个变量记录了在一次分裂循环中，那些可以再继续进行分裂的节点中包含的特征点数目和其句柄
            vSizeAndPointerToNode.clear();

            // 将目前的子区域进行划分
            // 开始遍历列表中所有的提取器节点，并进行分解或者保留
            while (lit != lNodes.end())
            {
                // 如果提取器节点只有一个特征点，
                if (lit->bNoMore)
                {
                    // If node only contains one point do not subdivide and continue
                    // 那么就没有必要再进行细分了
                    lit++;
                    
                    // 跳过当前节点，继续下一个
                    continue;
                }
                else
                {
                    // If more than one point, subdivide
                    // 如果当前的提取器节点具有超过一个的特征点，那么就要进行继续分裂
                    ExtractorNode n1, n2, n3, n4;

                    // 再细分成四个子区域
                    lit->DivideNode(n1, n2, n3, n4);

                    // Add childs if they contain points
                    // 如果这里分出来的子区域中有特征点，那么就将这个子区域的节点添加到提取器节点的列表中
                    // 注意这里的条件是，有特征点即可
                    if (n1.vKeys.size() > 0)
                    {
                        // 注意这里也是添加到列表前面的
                        lNodes.push_front(n1);

                        // 再判断其中子提取器节点中的特征点数目是否大于1
                        if (n1.vKeys.size() > 1)
                        {
                            // 如果有超过一个的特征点，那么待展开的节点计数加1
                            nToExpand++;

                            // 保存这个特征点数目和节点指针的信息
                            vSizeAndPointerToNode.push_back(std::make_pair(n1.vKeys.size(), &lNodes.front()));

                            //?这个访问用的句柄貌似并没有用到？
                            // lNodes.front().lit 和前面的迭代的lit 不同，只是名字相同而已
                            // lNodes.front().lit是node结构体里的一个指针用来记录节点的位置
                            // 迭代的lit 是while循环里作者命名的遍历的指针名称
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    // 后面的操作都是相同的，这里不再赘述
                    if (n2.vKeys.size() > 0)
                    {
                        lNodes.push_front(n2);
                        if (n2.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(std::make_pair(n2.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n3.vKeys.size() > 0)
                    {
                        lNodes.push_front(n3);
                        if (n3.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(std::make_pair(n3.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n4.vKeys.size() > 0)
                    {
                        lNodes.push_front(n4);
                        if (n4.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(std::make_pair(n4.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    // 当这个母节点expand之后就从列表中删除它了，能够进行分裂操作说明至少有一个子节点的区域中特征点的数量是>1的
                    //  分裂方式是后加的节点先分裂，先加的后分裂
                    lit = lNodes.erase(lit);

                    // 继续下一次循环，其实这里加不加这句话的作用都是一样的
                    continue;
                } // 判断当前遍历到的节点中是否有超过一个的特征点
            }     // 遍历列表中的所有提取器节点

            // Finish if there are more nodes than required features or all nodes contain just one point
            // 停止这个过程的条件有两个，满足其中一个即可：
            // 1、当前的节点数已经超过了要求的特征点数
            // 2、当前所有的节点中都只包含一个特征点
            if ((int)lNodes.size() >= N            // 判断是否超过了要求的特征点数
                || (int)lNodes.size() == prevSize) // prevSize中保存的是分裂之前的节点个数，如果分裂之前和分裂之后的总节点个数一样，说明当前所有的
                                                   // 节点区域中只有一个特征点，已经不能够再细分了
            {
                // 停止标志置位
                bFinish = true;
            }

            // Step 6 当再划分之后所有的Node数大于要求数目时,就慢慢划分直到使其刚刚达到或者超过要求的特征点个数
            // 可以展开的子节点个数nToExpand x3，是因为一分四之后，会删除原来的主节点，所以乘以3
            /**
             * //?BUG 但是我觉得这里有BUG，虽然最终作者也给误打误撞、稀里糊涂地修复了
             * 注意到，这里的nToExpand变量在前面的执行过程中是一直处于累计状态的，如果因为特征点个数太少，跳过了下面的else-if，又进行了一次上面的遍历
             * list的操作之后，lNodes.size()增加了，但是nToExpand也增加了，尤其是在很多次操作之后，下面的表达式：
             * ((int)lNodes.size()+nToExpand*3)>N
             * 会很快就被满足，但是此时只进行一次对vSizeAndPointerToNode中点进行分裂的操作是肯定不够的；
             * 理想中，作者下面的for理论上只要执行一次就能满足，不过作者所考虑的“不理想情况”应该是分裂后出现的节点所在区域可能没有特征点，因此将for
             * 循环放在了一个while循环里面，通过再次进行for循环、再分裂一次解决这个问题。而我所考虑的“不理想情况”则是因为前面的一次对vSizeAndPointerToNode
             * 中的特征点进行for循环不够，需要将其放在另外一个循环（也就是作者所写的while循环）中不断尝试直到达到退出条件。
             * */
            else if (((int)lNodes.size() + nToExpand * 3) > N)
            {
                // 如果再分裂一次那么数目就要超了，这里想办法尽可能使其刚刚达到或者超过要求的特征点个数时就退出
                // 这里的nToExpand和vSizeAndPointerToNode不是一次循环对一次循环的关系，而是前者是累计计数，后者只保存某一个循环的
                // 一直循环，直到结束标志位被置位
                while (!bFinish)
                {
                    // 获取当前的list中的节点个数
                    prevSize = lNodes.size();

                    // 保留那些还可以分裂的节点的信息, 这里是深拷贝
                    std::vector<std::pair<int, ExtractorNode *>> vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                    // 清空
                    vSizeAndPointerToNode.clear();

                    // 对需要划分的节点进行排序，对pair对的第一个元素进行排序，默认是从小到大排序
                    // 优先分裂特征点多的节点，使得特征点密集的区域保留更少的特征点
                    //! 注意这里的排序规则非常重要！会导致每次最后产生的特征点都不一样。建议使用 stable_sort
                    std::sort(vPrevSizeAndPointerToNode.begin(), vPrevSizeAndPointerToNode.end());

                    // 遍历这个存储了pair对的vector，注意是从后往前遍历
                    for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--)
                    {
                        ExtractorNode n1, n2, n3, n4;
                        // 对每个需要进行分裂的节点进行分裂
                        vPrevSizeAndPointerToNode[j].second->DivideNode(n1, n2, n3, n4);

                        // Add childs if they contain points
                        // 其实这里的节点可以说是二级子节点了，执行和前面一样的操作
                        if (n1.vKeys.size() > 0)
                        {
                            lNodes.push_front(n1);
                            if (n1.vKeys.size() > 1)
                            {
                                // 因为这里还有对于vSizeAndPointerToNode的操作，所以前面才会备份vSizeAndPointerToNode中的数据
                                // 为可能的、后续的又一次for循环做准备
                                vSizeAndPointerToNode.push_back(std::make_pair(n1.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n2.vKeys.size() > 0)
                        {
                            lNodes.push_front(n2);
                            if (n2.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(std::make_pair(n2.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n3.vKeys.size() > 0)
                        {
                            lNodes.push_front(n3);
                            if (n3.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(std::make_pair(n3.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n4.vKeys.size() > 0)
                        {
                            lNodes.push_front(n4);
                            if (n4.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(std::make_pair(n4.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }

                        // 删除母节点，在这里其实应该是一级子节点
                        lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                        // 判断是是否超过了需要的特征点数？是的话就退出，不是的话就继续这个分裂过程，直到刚刚达到或者超过要求的特征点个数
                        // 作者的思想其实就是这样的，再分裂了一次之后判断下一次分裂是否会超过N，如果不是那么就放心大胆地全部进行分裂（因为少了一个判断因此
                        // 其运算速度会稍微快一些），如果会那么就引导到这里进行最后一次分裂
                        if ((int)lNodes.size() >= N)
                            break;
                    } // 遍历vPrevSizeAndPointerToNode并对其中指定的node进行分裂，直到刚刚达到或者超过要求的特征点个数

                    // 这里理想中应该是一个for循环就能够达成结束条件了，但是作者想的可能是，有些子节点所在的区域会没有特征点，因此很有可能一次for循环之后
                    // 的数目还是不能够满足要求，所以还是需要判断结束条件并且再来一次
                    // 判断是否达到了停止条件
                    if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize)
                        bFinish = true;
                } // 一直进行nToExpand累加的节点分裂过程，直到分裂后的nodes数目刚刚达到或者超过要求的特征点数目
            }     // 当本次分裂后达不到结束条件但是再进行一次完整的分裂之后就可以达到结束条件时
        }         // 根据兴趣点分布,利用4叉树方法对图像进行划分区域

        // Retain the best point in each node
        // Step 7 保留每个区域响应值最大的一个兴趣点
        // 使用这个vector来存储我们感兴趣的特征点的过滤结果
        std::vector<cv::KeyPoint> vResultKeys;

        // 调整容器大小为要提取的特征点数目
        vResultKeys.reserve(camera_ptr->nFeatures);

        // 遍历这个节点链表
        for (std::list<ExtractorNode>::iterator lit = lNodes.begin(); lit != lNodes.end(); lit++)
        {
            // 得到这个节点区域中的特征点容器句柄
            std::vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;

            // 得到指向第一个特征点的指针，后面作为最大响应值对应的关键点
            cv::KeyPoint *pKP = &vNodeKeys[0];

            // 用第1个关键点响应值初始化最大响应值
            float maxResponse = pKP->response;

            // 开始遍历这个节点区域中的特征点容器中的特征点，注意是从1开始哟，0已经用过了
            for (size_t k = 1; k < vNodeKeys.size(); k++)
            {
                // 更新最大响应值
                if (vNodeKeys[k].response > maxResponse)
                {
                    // 更新pKP指向具有最大响应值的keypoints
                    pKP = &vNodeKeys[k];
                    maxResponse = vNodeKeys[k].response;
                }
            }

            // 将这个节点区域中的响应值最大的特征点加入最终结果容器
            vResultKeys.push_back(*pKP);
        }

        // 返回最终结果容器，其中保存有分裂出来的区域中，我们最感兴趣、响应值最大的特征点
        return vResultKeys;
    }

    void ORBFeature::computeOrientation(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, const std::vector<int> &umax)
    {
        // 遍历所有的特征点
        for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(), keypointEnd = keypoints.end();
                                                            keypoint != keypointEnd; ++keypoint)
        {
            // 调用IC_Angle 函数计算这个特征点的方向
            keypoint->angle = IC_Angle(image,        // 特征点所在的图层的图像
                                       keypoint->pt, // 特征点在这张图像中的坐标
                                       umax);        // 每个特征点所在图像区块的每行的边界 u_max 组成的vector
        }
    }

    float ORBFeature::IC_Angle(const cv::Mat &image, cv::Point2f pt, const std::vector<int> &u_max)
    {
        // 图像的矩，前者是按照图像块的y坐标加权，后者是按照图像块的x坐标加权
        int m_01 = 0, m_10 = 0;

        // 获得这个特征点所在的图像块的中心点坐标灰度值的指针center
        const uchar *center = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));

        // Treat the center line differently, v=0
        // 这条v=0中心线的计算需要特殊对待
        // 后面是以中心行为对称轴，成对遍历行数，所以PATCH_SIZE必须是奇数
        for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)

            // 注意这里的center下标u可以是负的！中心水平线上的像素按x坐标（也就是u坐标）加权
            m_10 += u * center[u];

        // Go line by line in the circular patch
        // 这里的step1表示这个图像一行包含的字节总数。参考[https://blog.csdn.net/qianqing13579/article/details/45318279]
        int step = (int)image.step1();

        // 注意这里是以v=0中心线为对称轴，然后对称地每成对的两行之间进行遍历，这样处理加快了计算速度
        for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
        {
            // Proceed over the two lines
            // 本来m_01应该是一列一列地计算的，但是由于对称以及坐标x,y正负的原因，可以一次计算两行
            int v_sum = 0;

            // 获取某行像素横坐标的最大范围，注意这里的图像块是圆形的！
            int d = u_max[v];

            // 在坐标范围内挨个像素遍历，实际是一次遍历2个
            //  假设每次处理的两个点坐标，中心线下方为(x,y),中心线上方为(x,-y)
            //  对于某次待处理的两个点：m_10 = Σ x*I(x,y) =  x*I(x,y) + x*I(x,-y) = x*(I(x,y) + I(x,-y))
            //  对于某次待处理的两个点：m_01 = Σ y*I(x,y) =  y*I(x,y) - y*I(x,-y) = y*(I(x,y) - I(x,-y))
            for (int u = -d; u <= d; ++u)
            {
                // 得到需要进行加运算和减运算的像素灰度值
                // val_plus：在中心线下方x=u时的的像素灰度值
                // val_minus：在中心线上方x=u时的像素灰度值
                int val_plus = center[u + v * step], val_minus = center[u - v * step];

                // 在v（y轴）上，2行所有像素灰度值之差
                v_sum += (val_plus - val_minus);

                // u轴（也就是x轴）方向上用u坐标加权和（u坐标也有正负符号），相当于同时计算两行
                m_10 += u * (val_plus + val_minus);
            }

            // 将这一行上的和按照y坐标加权
            m_01 += v * v_sum;
        }

        // 为了加快速度还使用了fastAtan2()函数，输出为[0,360)角度，精度为0.3°
        return cv::fastAtan2((float)m_01, (float)m_10);
    }

    void ORBFeature::computeDescriptors(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors, const std::vector<cv::Point> &pattern)
    {
        // 清空保存描述子信息的容器
        descriptors = cv::Mat::zeros((int)keypoints.size(), 32, CV_8UC1);

        // 开始遍历特征点
        for (size_t i = 0; i < keypoints.size(); i++)
            // 计算这个特征点的描述子
            computeOrbDescriptor(keypoints[i],             // 要计算描述子的特征点
                                 image,                    // 以及其图像
                                 &pattern[0],              // 随机点集的首地址
                                 descriptors.ptr((int)i)); // 提取出来的描述子的保存位置
    }

    void ORBFeature::computeOrbDescriptor(const cv::KeyPoint &kpt, const cv::Mat &img, const cv::Point *pattern, uchar *desc)
    {
        // 得到特征点的角度，用弧度制表示。其中kpt.angle是角度制，范围为[0,360)度
        float angle = (float)kpt.angle * M_PI / 180;

        // 计算这个角度的余弦值和正弦值
        float a = (float)cos(angle), b = (float)sin(angle);

        // 获得图像中心指针
        const uchar *center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));

        // 获得图像的每行的字节数
        const int step = (int)img.step;

        // 原始的BRIEF描述子没有方向不变性，通过加入关键点的方向来计算描述子，称之为Steer BRIEF，具有较好旋转不变特性
        // 具体地，在计算的时候需要将这里选取的采样模板中点的x轴方向旋转到特征点的方向。
        // 获得采样点中某个idx所对应的点的灰度值,这里旋转前坐标为(x,y), 旋转后坐标(x',y')，他们的变换关系:
        //  x'= xcos(θ) - ysin(θ),  y'= xsin(θ) + ycos(θ)
        //  下面表示 y'* step + x'
        #define GET_VALUE(idx) center[cvRound(pattern[idx].x * b + pattern[idx].y * a) * step + cvRound(pattern[idx].x * a - pattern[idx].y * b)]

        // brief描述子由32*8位组成
        // 其中每一位是来自于两个像素点灰度的直接比较，所以每比较出8bit结果，需要16个随机点，这也就是为什么pattern需要+=16的原因
        for (int i = 0; i < 32; ++i, pattern += 16)
        {

            int t0,  // 参与比较的第1个特征点的灰度值
                t1,  // 参与比较的第2个特征点的灰度值
                val; // 描述子这个字节的比较结果，0或1

            t0 = GET_VALUE(0);
            t1 = GET_VALUE(1);
            val = t0 < t1; // 描述子本字节的bit0
            t0 = GET_VALUE(2);
            t1 = GET_VALUE(3);
            val |= (t0 < t1) << 1; // 描述子本字节的bit1
            t0 = GET_VALUE(4);
            t1 = GET_VALUE(5);
            val |= (t0 < t1) << 2; // 描述子本字节的bit2
            t0 = GET_VALUE(6);
            t1 = GET_VALUE(7);
            val |= (t0 < t1) << 3; // 描述子本字节的bit3
            t0 = GET_VALUE(8);
            t1 = GET_VALUE(9);
            val |= (t0 < t1) << 4; // 描述子本字节的bit4
            t0 = GET_VALUE(10);
            t1 = GET_VALUE(11);
            val |= (t0 < t1) << 5; // 描述子本字节的bit5
            t0 = GET_VALUE(12);
            t1 = GET_VALUE(13);
            val |= (t0 < t1) << 6; // 描述子本字节的bit6
            t0 = GET_VALUE(14);
            t1 = GET_VALUE(15);
            val |= (t0 < t1) << 7; // 描述子本字节的bit7

            // 保存当前比较的出来的描述子的这个字节
            desc[i] = (uchar)val;
        }

        // 为了避免和程序中的其他部分冲突在，在使用完成之后就取消这个宏定义
        #undef GET_VALUE
    }
}