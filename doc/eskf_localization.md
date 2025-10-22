# Leg-KILO 中的 ESKF 定位实现

本文结合 Leg-KILO 源码对误差状态扩展卡尔曼滤波器（ESKF）的定位流程进行拆解，并以数学公式串联各模块，帮助读者理解系统如何融合 LiDAR、IMU 及腿部运动学信息。

## 状态向量与误差注入
ESKF 维护 30 维状态，将旋转、位置、速度与 IMU/运动学相关量紧耦合：
\[
\mathbf{x} = \big[\boldsymbol{\theta},\; \mathbf{p},\; \mathbf{v},\; \mathbf{b}_a,\; \mathbf{b}_\omega,\; \mathbf{g},\; \mathbf{a},\; \boldsymbol{\omega},\; \mathbf{b}_v,\; \mathbf{c}\big]^\top.
\]
其中 \(\boldsymbol{\theta}\) 为姿态误差的旋转向量，\(\mathbf{p}\) 为机体位置，\(\mathbf{v}\) 为线速度，\(\mathbf{b}_a/\mathbf{b}_\omega\) 为 IMU 偏置，\(\mathbf{g}\) 为重力向量，\(\mathbf{a}/\boldsymbol{\omega}\) 为当前估计的 IMU 读数，\(\mathbf{b}_v\) 为运动学速度偏置，\(\mathbf{c}\) 为接触点位置。【F:legkilo/src/core/slam/eskf.h†L15-L109】

在误差注入阶段，滤波器通过李代数指数映射修正姿态，其余量线性叠加：
\[
\begin{aligned}
\mathbf{R} &\leftarrow \mathbf{R}\,\exp(\delta\boldsymbol{\theta}),\\
\mathbf{p} &\leftarrow \mathbf{p} + \delta\mathbf{p},\quad \mathbf{v} \leftarrow \mathbf{v} + \delta\mathbf{v},\\
\mathbf{b}_a &\leftarrow \mathbf{b}_a + \delta\mathbf{b}_a,\; \ldots,\; \mathbf{c} \leftarrow \mathbf{c} + \delta\mathbf{c}.
\end{aligned}
\]
该操作由 `State::operator+=` 实现，对应的误差提取由 `State::operator-` 完成。【F:legkilo/src/core/slam/eskf.cc†L5-L45】

## 初始化与过程噪声
首帧通过 IMU 或 IMU+运动学对重力与陀螺偏置求均值，并设置初始协方差及过程噪声矩阵：
\[
\mathbf{g}_0 = -\frac{\overline{\mathbf{a}}}{\lVert \overline{\mathbf{a}} \rVert} g,\quad \mathbf{b}_{\omega,0} = \overline{\boldsymbol{\omega}},\quad \mathbf{P}_0 = 10^{-6}\mathbf{I}.
\]
初始化完成后调用 `initProcessCovQ()`，将速度、IMU 读数、偏置、接触等子状态的过程方差填入对角块，为后续协方差传播提供随机游走噪声模型。【F:legkilo/src/preprocess/state_initial.hpp†L30-L117】【F:legkilo/src/core/slam/eskf.cc†L47-L62】

## 预测模型
ESKF 预测方程直接来自 `getFunctionf` 与 `getFx`：
\[
\begin{aligned}
\boldsymbol{\theta}_{k+1} &= \boldsymbol{\theta}_k + \Delta t\,\boldsymbol{\omega},\\
\mathbf{p}_{k+1} &= \mathbf{p}_k + \Delta t\,\mathbf{v}_k,\\
\mathbf{v}_{k+1} &= \mathbf{v}_k + \Delta t\,(\mathbf{R}_k\mathbf{a} + \mathbf{g}).
\end{aligned}
\]
雅可比 \(\mathbf{F}\) 同时考虑角速度对姿态的指数积分、速度对位置的影响，以及加速度、重力、IMU 读数对速度的耦合项：
\[
\mathbf{F} = \mathbf{I} +
\begin{bmatrix}
-\Delta t\,\boldsymbol{\omega}_\times & \mathbf{0} & \mathbf{0} & \cdots & \Delta t\,\mathbf{I} \\
\mathbf{0} & \mathbf{I} & \Delta t\,\mathbf{I} & \cdots & \mathbf{0} \\
-\Delta t\,\mathbf{R}\mathbf{a}_\times & \mathbf{0} & \mathbf{I} & \cdots & \Delta t\,\mathbf{R}
\end{bmatrix}.
\]
预测函数在需要时分离“只传递状态”与“只传播协方差”两种调用，以保证在任意时间戳到来时状态与协方差同步：
\[
\mathbf{x}_{k+1} = f(\mathbf{x}_k, \Delta t),\quad \mathbf{P}_{k+1} = \mathbf{F}\mathbf{P}_k\mathbf{F}^\top + (\Delta t)^2\mathbf{Q}.
\]
这些操作由 `predict` 按需启用状态推进 (`prop_state`) 与协方差传播 (`prop_cov`) 完成。【F:legkilo/src/core/slam/eskf.cc†L64-L89】

## 点到面观测的构建
针对每个下采样点，系统先按外参与当前位姿投影到世界系，并传播点协方差：
\[
\mathbf{p}_w = \mathbf{R}\big(\mathbf{R}_{ex}\mathbf{p}_b + \mathbf{t}_{ex}\big) + \mathbf{p},\quad
\mathbf{\Sigma}_w = \mathbf{R}\mathbf{R}_{ex}\mathbf{\Sigma}_b\mathbf{R}_{ex}^\top\mathbf{R}^\top + \mathbf{R}[\mathbf{p}_i]_\times \mathbf{P}_{RR}[\mathbf{p}_i]_\times^\top\mathbf{R}^\top + \mathbf{P}_{pp}.
\]
随后在体素八叉树中寻找平面，将点到面的符号距离
\[
d = \mathbf{n}^\top \mathbf{p}_w + b
\]
写入残差向量，并构造雅可比
\[
\mathbf{h} = \big[(\mathbf{p}_w - \mathbf{c})^\top[\mathbf{R}]_\times\mathbf{n},\; \mathbf{n}^\top\big]
\]
以及测量噪声
\[
R = \lambda\left(\mathbf{J}_{nq}\,\mathbf{\Sigma}_{\pi}\,\mathbf{J}_{nq}^\top + \mathbf{n}^\top \mathbf{\Sigma}_w \mathbf{n}\right),
\]
其中 \(\lambda\) 为配置中的比例系数。以上流程在 `predictUpdatePoint` 中完成。【F:legkilo/src/core/slam/KILO.cc†L108-L232】

## LiDAR 更新方程
当有有效平面匹配时，`updateByPoints` 根据观测维度选择标量或矩阵形式的卡尔曼增益：
\[
\mathbf{K} = \mathbf{P}\mathbf{H}^\top(\mathbf{H}\mathbf{P}\mathbf{H}^\top + \mathbf{R})^{-1},\quad
\delta\mathbf{x} = \mathbf{K}\mathbf{z},\quad
\mathbf{P} \leftarrow \mathbf{P} - \mathbf{K}\mathbf{H}\mathbf{P}.
\]
其中 \(\mathbf{H}\) 由上节构建的点到面雅可比组成，\(\mathbf{z}\) 为残差向量。单点观测时使用快速标量更新，多点情况下退化为常规矩阵形式。【F:legkilo/src/core/slam/eskf.cc†L91-L123】

## IMU 模式下的观测
纯 IMU 模式中，每条测量构成 6 维观测：
\[
\mathbf{z}_{imu} =
\begin{bmatrix}
\tfrac{g}{\|\mathbf{a}_{m}\|}\mathbf{a}_{m} - (\mathbf{a} + \mathbf{b}_a) \\
\boldsymbol{\omega}_{m} - (\boldsymbol{\omega} + \mathbf{b}_\omega)
\end{bmatrix},
\]
并附带各向异性的噪声方差。观测雅可比仅作用于 IMU 偏置与读数分量，`updateByImu` 通过等效的协方差块求取卡尔曼增益并更新状态。【F:legkilo/src/core/slam/KILO.cc†L235-L257】【F:legkilo/src/core/slam/eskf.cc†L125-L135】

## 运动学+IMU 紧耦合
当腿部接触有效时，系统将 IMU 观测与零速度约束拼接：
\[
\mathbf{z}_{kin,i} = -\mathbf{v} - \mathbf{R}\big(\boldsymbol{\omega}_\times\mathbf{p}_{f_i} + \mathbf{v}_{f_i}\big),
\]
其中 \(\mathbf{p}_{f_i}, \mathbf{v}_{f_i}\) 为足端位置与速度。对应雅可比在姿态、速度、IMU 读数及运动学偏置上具有非零项，协方差对角填入足端速度噪声。`updateByKinImu` 以标准 EKF 形式融合整批观测。【F:legkilo/src/core/slam/KILO.cc†L260-L313】【F:legkilo/src/core/slam/eskf.cc†L137-L145】

## 时序对齐与循环策略
`process` 函数按点的时间偏移排序扫描，并在每个时间桶内：先预测到该时刻、插入所有早于此刻的 IMU 或 Kin+IMU 观测，再执行点到面更新。如此即可在一帧 LiDAR 扫描内实现细粒度的时序对齐与多传感器滚动融合。【F:legkilo/src/core/slam/KILO.cc†L316-L398】

通过以上步骤，Leg-KILO 的 ESKF 实现了多传感器的紧耦合定位，在保持实时性的同时充分利用地图几何约束、惯导动态与腿部接触信息。
