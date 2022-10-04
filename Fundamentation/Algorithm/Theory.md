### ORB 特征

##### FAST 关键点检测
**1.** 在图像中选取像素 $p$, 亮度为 $I_p$;
**2.** 设置一个阈值 $T$;
**3.** 以像素 $p$ 为中心, 选取半径为 $3$ 的圆上 $16$ 个像点;
**4.** 满足圆上有连续的 $N$ 个点的亮度： $\ge I_p+T$ or $\le I_p-T$, $p$为特征点.

注：一般检测之后采用 **Non-maximal suppression** 保留响应极大值角点.

##### 旋转不变性
**1.** 定义图像块矩
$$
m_{pq} = \sum _{x,y \in B}x^py^qI(x,y) \quad p,q=\{0,1\}.
$$
**2.** 图像块质心
$$
C = \left(\frac{m_{10}}{m_{00}}, 
\frac{m_{01}}{m_{00}} \right).
$$
**3.** 连接图像块几何中心 $O$ 和质心 $C$, 得到方向向量 $\overrightarrow {OC}$, 定义特征点方向
$$
\theta = \arctan \left(\frac{m_{01}}{m_{10}} \right).
$$

##### BRIEF 描述子
一种 **Binary** 描述子, 通过比较关键点附近随机像素大小获得, 一般为 $128$ 维 $0$ 和 $1$ 组成的向量.

### 特征匹配
**1.** Brute-force Matcher
**2.** Hamming distance
**3.** FLANN