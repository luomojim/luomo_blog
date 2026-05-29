---
title: 君瞄、济瞄与 JLU 视觉算法仓库学习笔记
published: 2026-05-19
pinned: false
description: 记录君瞄、济瞄与 JLU 视觉算法仓库的架构、通信与自瞄算法学习笔记
tags: [robomaster, vision, auto-aim]
category: 算法
draft: false
date: 2026-05-19
pubDate: 2026-05-19
---

# 写在前面
除开前几年的君瞄是用的ros2之外，剩下的无论是sp_vision25还是jlu_vision26的仓库看下来，给我最大的感受就是：他们都开始丢掉ros2了。而我们ACE现在用的也还是基于ros2的节点来进行整个自瞄系统的管理，但这也给我们带来了挑战。不说学长学姐他们，我自己前阵子在研究怎么样在Unity的camera进行ros2的时候就遇到问题，进程间的通信就避免不了图像的拷贝传输，因为一定要走ros2的连接桥，才能通过ros2 tcp节点来订阅到引擎画面，而现在还是对齐工业相机的规格(1440 * 1080 * 150FPS),这样对内存的带宽要求也是极高的。同时ros2的高度依赖对应发行版也很沟槽......

诸如此类种种问题，在这十几天来，除了学习这几个自瞄项目的图像算法之外，更要熟悉他们的项目架构管理，了解他们的通信层等等，如果后续有机会的话就可以更彻底地卸下历史包袱，重构整套通信系统，实现零拷贝的跨进程或者进程内通信，减少硬件的负担，腾出更多的算力来支持其他组件。

# 君瞄

## 结构
君瞄算是一个非常经典的视觉算法开山鼻祖(我在许多视觉开源的项目都能看到君瞄的鸣谢)，同时也是基于ros2完成，作为很多ros2自瞄的脚手架，它提供了相机包和串口包在内的ros2节点，同时还有一套完整的自瞄打击系统，基本的结构就按照下面的节点图所示

```mermaid
flowchart LR
    camera["相机节点 /camera_info, /image_raw"] --> detector["armor_detector"]
    detector --> armors["/detector/armors"]
    detector --> detector_marker["/detector/marker"]
    detector --> detector_debug["/detector/debug_* 与调试图像"]
    armors --> tracker["armor_tracker"]
    tf["/tf, /tf_static"] --> tracker
    tracker --> target["/tracker/target"]
    tracker --> tracker_info["/tracker/info"]
    tracker --> tracker_marker["/tracker/marker"]
    reset["/tracker/reset"] --> tracker
    target --> serial["下游串口/云台控制节点"]
```
## armor_detector 节点

这里就是很标准的传统网络识别，接下来开始来介绍一下大致的流程。在前面通过订阅相机节点之后，就可以直接进入主要的流程

``` cpp
binary_img = preprocessImage(input);
lights_ = findLights(input, binary_img);
armors_ = matchLights(lights_);

if (!armors_.empty()) {
  classifier->extractNumbers(input, armors_);
  classifier->classify(armors_);
}

return armors_;
```

### 灯条识别

首先进行图片的预处理，通过进行二值化，这里二值化要使用灰度的方法，因为工业相机的 **动态范围不大**，为了得到白色数字 ，这导致灯条会出现过曝。所以选择灰度二值化这样就能得到比较完美的灯条

|                 原图                 |                    二值化                     |
| :--------------------------------: | :------------------------------------------: |
| ![raw](raw.png) | ![gray bin](gray_bin.png) |

来到了``findlights``函数，通过进行外接矩形的拟合来获得灯条的rbox,然后再经过一个`Light`函数来进行排序，把旋转矩形按照y进行重排序，最后取上下的中点来作为灯条的轴。之后进行几何过滤，通过计算宽高比和角度来筛选假灯条。来到颜色区分，通过累加rbox内部的靠近r和靠近b的像素，看看r和b哪个占比大。

### 灯条配对
首先先选择颜色相同的灯条进行一个复杂度为O(N^2)的二重循环遍历所有组合，其中去掉不同的颜色组合，再过滤掉两个灯条中间还有一个灯条的情况，还有长宽比，中心距离，连线的倾角进行一个筛子的处理。后续判断装甲板大小则使用中心距将其归一化，然后判断长度。

### 数字识别

| 原始数字 | 透视变换 | ROI | 二值化 |
| :----------------------------------------: | :------------------------------------------: | :----------------------------------------: | :----------------------------------------: |
| ![num raw](num_raw.png) | ![num warp](num_warp.png) | ![num roi](num_roi.png) | ![num bin](num_bin.png) |

如图片所示，首先从匹配好的灯条构建一个稍大一点的ROI区域，然后对其进行透视变换，之后缩放为高度固定的图像，之后根据装甲板大小截取中间的roi，之后进行转灰度然后二值化，得到清晰的二值化数字，如上面图4所示。之后送进下面的mlp识别网络里面。

### MLP网络
![[mlp.png]]

如上图所示，这是一个分类器使用了分类网络，同时这是一个多层感知机MLP，用来应付这种数字分类绰绰有余。
前面的roi是固定的大小20x28大小，然后送进多个隐藏层里面进行权重的计算，同时使用ReLU激活函数，这样就可以具备非线性表达能力，经过多次隐藏层的作用，最后变成了9个输出层，即每个类型的原始分数，再使用softmax函数把原始分数变成概率，得到了每个类别的概率，最后取最高值进行输出。

## armor_tracker 节点
来到这个部分，就可以开始处理跟踪装甲板整车估计了。同时这里也负责TF广播和消息发布。当收到了回调信号之后开始进行处理。

### EKF
![[ekf.png]]
如上图所示，我们先看状态向量  
$$
x=
\begin{bmatrix}
x_c & v_{x_c} & y_c & v_{y_c} & z_a & v_{z_a} & \psi & \dot\psi & r
\end{bmatrix}^T
$$
这里使用了一个九维卡尔曼滤波来进行整车状态的估计，状态向量包含了旋转中心的x,y,z和三个分速度，还有车辆旋转的角度和装甲板的朝向角度,以及一个车辆半径r。

#### 过程模型

这里为了简化计算，假设车辆进行匀速直线运动和匀速圆周运动
$$
x_c' = x_c + v_{x_c}\Delta t
$$
$$
y_c' = y_c + v_{y_c}\Delta t
$$
$$
z_a' = z_a + v_{z_a}\Delta t
$$
$$
\psi' = \psi + \dot\psi\Delta t
$$

#### 观测模型
这里使用四个观测量来得到观测向量：
$$
z=
\begin{bmatrix}
x_a & y_a & z_a & \psi_a
\end{bmatrix}^T
$$
然后观测模型中的装甲板的xy是直接使用整车中心减去r的正余弦反推出来
$$
x_a=x_c-r\cos\psi$$
$$y_a=y_c-r\sin\psi
$$
因为这里出现了正余弦yaw,所以是非线性的，所以没法用一个常数矩阵H来解决，所以需要在估计点附近进行一次一阶泰勒展开，变成近似线性的H观测矩阵

$$
\frac{\partial x_a}{\partial x_c}=1,\quad
\frac{\partial x_a}{\partial \psi}=r\sin\psi,\quad
\frac{\partial x_a}{\partial r}=-\cos\psi
$$
$$
\frac{\partial y_a}{\partial y_c}=1,\quad
\frac{\partial y_a}{\partial \psi}=-r\cos\psi,\quad
\frac{\partial y_a}{\partial r}=-\sin\psi
$$

#### 噪声矩阵
这里过程噪声协方差用了一个连续时间白噪声加速度模型

$$
Q_{pos,vel}
=
\sigma^2
\begin{bmatrix}
\frac{\Delta t^4}{4} & \frac{\Delta t^3}{2}\\
\frac{\Delta t^3}{2} & \Delta t^2
\end{bmatrix}
$$

假设加速度是零均值白噪声，功率谱密度为 $σ^2$。对这个随机加速度积分一次得到速度扰动，积分两次得到位置扰动，经过Δt时间后，位置和速度的相关性就构成了上面这个2×2矩阵。

而观测噪声矩阵则根据一个x乘以各个坐标xyz观测，而yaw的噪声则是固定的。

$$
\mathbf{R}
=
\mathrm{diag}\!\left(
|x \cdot z_0|,\;
|x \cdot z_1|,\;
|x \cdot z_2|,\;
r_{\text{yaw}}
\right)
$$

#### 预测更新
预测步：
$$
x_{k|k-1}=f(x_{k-1|k-1})
$$
$$
P_{k|k-1}=F P_{k-1|k-1} F^T+Q
$$

更新步：
$$
K=P H^T(HPH^T+R)^{-1}
$$
$$
x_{k|k}=x_{k|k-1}+K(z-h(x_{k|k-1}))
$$
$$
P_{k|k}=(I-KH)P_{k|k-1}
$$
