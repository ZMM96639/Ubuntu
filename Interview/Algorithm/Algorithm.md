## Non-linear Least Squares

##### Problem:

$$\min_xF(x) = \frac{1}{2}\Vert f(x)\Vert_2^2.$$

**Analytic Solution :**
$$\frac{dF}{dx} = 0.$$

**Numeric Solution :**

1. 给定 ***initial value*** $x_0$;
2. 对于第 $k$ 次 ***iteration***, 寻找一个 ***increment*** $\Delta x$,使得 $\Vert f(x_k+\Delta x_k)\Vert_2^2$ 达到 ***min***;
3. 若 $\Delta x$ 足够小, ***Stop***. ***Otherwise***, $x_{k+1} = x_k + \Delta x_k$. 

## g2o using tutorial

#### 1. Theory

##### *Gauss-Newton* :

1. 给定 ***initial value*** $x_0$;
2. 对于第 $k$ 次 ***iteration***, 求出当前的 ***Jacobian Matrix*** 和 ***Error*** $f(x_k)$;
3. 求解 ***increment function*** : $H\Delta x_k = g$;
4. 若 $\Delta x$ 足够小, ***Stop***. ***Otherwise***, $x_{k+1} = x_k + \Delta x_k$.

$$ f(x + \Delta x) \Rightarrow f(x) + \pmb J(x)\Delta x.$$ $$\Delta x^* = \argmin_{\Delta x} \frac{1}{2} \Vert {f(x)+\pmb J(x)\Delta x}\Vert^2. $$ $$ \begin{align}
\frac{1}{2}{\Vert f(x) + \pmb J(x)\Delta x\Vert}^2 & = \frac{1}{2} \left(f(x) + \pmb J(x)\Delta x \right)^T\left(f(x) + \pmb J(x)\Delta x\right) \\ & = \frac{1}{2}\left(\Vert f(x)\Vert^2_2 + 2f(x)^T\pmb J(x)\Delta x + \Delta x^T\pmb J(x)^T\pmb J(x)\Delta x\right)
\end{align} $$

##### *Derivatives subject to* $\Delta x$ : 

$$\pmb J(x)^T\pmb J(x)\Delta x = -\pmb J(x)^Tf(x).$$ $$\pmb H\Delta x = \pmb g.$$

注：实际数据中 $\pmb {J^TJ}$ 是半正定的, $\pmb {J^TJ}$ 为 ***Singular matrix Or ill-condition***, 导致算法不收敛.

##### *Levenberg-Marquadt* :

1. 给定 ***initial value*** $x_0$, 以及初始优化半径 $\mu$ ;
2. 对于第 $k$ 次 ***iteration***, 在 ***Gauss-Newton*** 基础上加上  ***Trust Region***, 求解：
$$\min _{\Delta x_k}\frac{1}{2}\Vert f(x_k) + \pmb J(x_k)\Delta x\Vert^2 + \frac{\lambda}{2}\Vert\pmb D\Delta x\Vert^2, s.t. \Vert D\Delta x_k\Vert^2 \le \mu.$$
3. 计算 $\rho$ :
$$\rho = \frac {f(x + \Delta x) - f(x)}{\pmb J(x)\Delta x}.$$ 
4.若 $\rho\gt\frac{3}{4}$, 则 $\mu = 2\mu$ ;若 $\rho\lt\frac{1}{4}$, 则 $\mu = 0.5\mu$ ;
5. $x_{k+1} = x_k + \Delta x.$
$$\left(\pmb H + \lambda \Delta \pmb D^T\pmb D\right)\Delta x = \pmb g.$$ $$\left(\pmb H + \lambda \pmb I\right)\Delta x = \pmb g.$$

注：当 $\lambda$ 比较小时, `Levenberg-Marquadt` $\to$ `Gauss-Newton`; 当 $\lambda$ 比较大时, `Levenberg-Marquadt` $\to$ `最速下降法`.

#### 2. Procedure

- **定义顶点和边的类型；**
- **构建图；**
- **选择优化算法；**
- **调用 g2o 进行优化。**

#### 3. Vertex(优化变量)

##### 一般继承至 Base Vertex.

- **顶点初始化 : virtual void** `setToOriginImpl()` **override;** // *_estimate = 0.*
- **顶点更新函数 : virtual void** `oplusImpl(const double *update)`**override;** // $x_{k+1} = x_k+\Delta x$.

- **读盘函数 : virtual bool** `read(std::istream &in)` **override;**
- **存盘函数 : virtual bool** `write(std::ostream &out)` **const override;**

#### 4. Edge(误差项)

##### 可继承至 BaseUnaryEdge, BaseBinaryEdge, BaseMultiEdge.

- **误差计算函数 : virtual void** `computeError()` **override;** // **_error = _measurement - pos.*
- **雅可比矩阵计算函数 : virtual void** `linearizeOplus()`**override;** // **Optional :** *Numeric Derivatives or Analytic Derivatives.*

- **读盘函数 : virtual bool** `read(std::istream &in)` **override;**
- **存盘函数 : virtual bool** `write(std::ostream &out)` **const override;**

#### 5. Implement

- **确定线性求解器类型** LinearSolver
    ```
    // PoseDim: 待优化变量; LandmarkDim: 误差维数
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<PoseDim, LandmarkDim>> Block;
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    ```
- **确定块求解器类型** BlockSolver
    ```
    Block *solver_ptr = new Block(linearSolver);  
    ```
- **确定总求解器** Solver 
    ```
    // GN、LM or DogLeg
    auto solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    ```
- **确定图模型** SparseOptimizer
    ```
    g2o::SparseOptimizer optimizer; // 图模型
    optimizer.setAlgorithm(solver); // 设置求解器
    optimizer.setVerbose(true);    // 打开调试输出
    ```
- **添加顶点和边**
    ```
    // 添加顶点
    myVertex *v = new myVertex();
    v->setId(0);
    v->setEstimate(/* data */);
    optimizer.addVertex(v);

    // 添加边
     for (int i = 0; i < n; ++i)
        {
            myEdge *edge = new myEdge(/* data */);
            edge->setId(i);
            edge->setVertex(0, v);  // 设置连接的顶点
            edge->setMeasurement( /* _measurement */ ); // 观测值
            edge->setInformation( /* _information */ ); // 信息矩阵：协方差矩阵之逆
            optimizer.addEdge(edge);
        }
    ```
- **执行优化**
    ```
    1. LinearSolver
    2. BlockSolver
    3. Solver
    4. SparseOptimizer
    5. addVertex(Id, V)
    6. addEdge(Id, V, Measurement, Information, Edge)
    7. InitializeOptimization
    8. Iteration
    ```


## Kalman filtering
#### 1. Time Update("Predict")

- **Project the state ahead**
$$ \hat x_k^- = A\hat x_{k-1} + Bu_{k-1}$$
- **Project the error covariance ahead**
$$P_k^- = AP_{k-1}A^T + Q $$

#### 2. Measurement Update("Correct")

- **Compute the Kalman gain**
$$K_k = P_k^-H^T(HP_k^-H^T + R)^{-1}$$
- **Update estimate with measurement $z_k$**
$$\hat x_k = \hat x_k^- + K_k({z_k - H\hat x_k^-})$$
- **Update the error covariance**
$$P_k = (I - K_kH)P_k^-$$

#### 3. Kalman Filter Algorithm Reference Terms


|Variable|Introduction|Typy||
|--------|--------:|:--------:|:--------:|
|x|state variable|n x 1 column vector|Output|
|P|state covariance matrix|n x n matrix|Output|
|z|measurement|m x 1 column vector|Input|
|A|state transition matrix|n x n matrix|State-transition Model|
|B|control matrix|n x l matrix|Control-input model|
|H|state-to-measurement matrix|m x n matrix|Observation Model|
|R|measurement covariance matrix|m x m matrix|Input|
|Q|process noise covarivance matrix|n x n matrix|System Model|
|K|Kalman Gain|n x m|Internal|
|z-Hx|measurement residual|||

It implements the algorithm directly as found in [An Introduction to the Kalman Filter](http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf).

More reference:
[Zhamemo](https://github.com/Zhamemo/Ubuntu/tree/master/Interview/Algorithm)
[Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter)
[Kalman Filter Explained Simply](https://thekalmanfilter.com/kalman-filter-explained-simply/)
[How a Kalman filter works in pictures](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)



