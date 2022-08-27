## g2o using tutorial

### 1. Theory

#### *Gauss-Newton* :
$$ f(x + \Delta x) \Rightarrow f(x) + \pmb J(x)\Delta x.$$ $$\Delta x^* = \argmin_{\Delta x} \frac{1}{2} \Vert {f(x)+\pmb J(x)\Delta x}\Vert^2. $$ $$\frac{1}{2}{\Vert f(x) + \pmb J(x)\Delta x\Vert}^2 = \frac{1}{2} \left(f(x) + \pmb J(x)\Delta x \right)^T\left(f(x) + \pmb J(x)\Delta x\right).$$ $$ = \frac{1}{2}\left(\Vert f(x)\Vert^2_2 + 2f(x)^T\pmb J(x)\Delta x + \Delta x^T\pmb J(x)^T\pmb J(x)\Delta x\right).$$

##### *Derivatives subject to* $\Delta x$ : $$\pmb J(x)^T\pmb J(x)\Delta x = -\pmb J(x)^Tf(x).$$ $$\pmb H\Delta x = \pmb g.$$

#### *Levenberg-Marquadt* :
$$\rho = \frac {f(x + \Delta x) - f(x)}{\pmb J(x)\Delta x}.$$ $$\min _{\Delta x_k}\frac{1}{2}\Vert f(x_k) + \pmb J(x_k)\Delta x\Vert^2 + \frac{\lambda}{2}\Vert\pmb D\Delta x\Vert^2.$$ $$\left(\pmb H + \lambda \Delta \pmb D^T\pmb D\right)\Delta x = \pmb g.$$ $$\left(\pmb H + \lambda \pmb I\right)\Delta x = \pmb g.$$

### 2. Procedure

- **定义顶点和边的类型；**
- **构建图；**
- **选择优化算法；**
- **调用 g2o 进行优化。**

### 3. Vertex(优化变量)
##### 一般继承至 Base Vertex.

- **顶点初始化 : virtual void** `setToOriginImpl()` **override;** // *_estimate = 0.*
- **顶点更新函数 : virtual void** `oplusImpl(const double *update)`**override;** // $x_{k+1} = x_k+\Delta x$.

- **读盘函数 : virtual bool** `read(std::istream &in)` **override;**
- **存盘函数 : virtual bool** `write(std::ostream &out)` **const override;**

### 4. Edge(误差项)
##### 可继承至 BaseUnaryEdge, BaseBinaryEdge, BaseMultiEdge.

- **误差计算函数 : virtual void** `computeError()` **override;** // **_error = _measurement - pos.*
- **雅可比矩阵计算函数 : virtual void** `linearizeOplus()`**override;** // **Optional :** *Numeric Derivatives or Analytic Derivatives.*

- **读盘函数 : virtual bool** `read(std::istream &in)` **override;**
- **存盘函数 : virtual bool** `write(std::ostream &out)` **const override;**

### 5. Implement
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
- **确定总求解器** solver 
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
    v->setEstimate(/* data */);
    v->setId(0);
    optimizer.addVertex(v);

    // 添加边
     for (int i = 0; i < n; ++i)
        {
            myEdge *edge = new myEdge(/* data */);
            edge->setId(i);
            edge->setVertex(0, v);  // 设置连接的顶点
            edge->setMeasurement( /* _measurement */ ); // 观测数值
            edge->setInformation( /* _information */ ); // 信息矩阵：协方差矩阵之逆
            optimizer.addEdge(edge);
        }
    ```
- **执行优化**

