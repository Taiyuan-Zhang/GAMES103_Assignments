# GAMES103_Assignments

repo for the assignments of GAMES103: 基于物理的计算机动画入门 by 王华民

这个项目记录了[GAMES103课程](http://games-cn.org/games103/)的PPT和作业


### 项目框架

> GAMES103课程PPT

> Lab *
>
> > lab*.pdf  											   ——>assignment description
> >
> > Lab*****_*.unitypackage 						 ——>assignment code file
> >
> > Zhang_Taiyuan_Lab*.unitypackage ——>my inplementation
> >
> > *.cs														——>my code

---

### 实验环境：

- Unity3D
- C# Script

### Lab1：Rigidbody Simulation（满分）

- 基于物理的刚体模拟（impulse-based方法）\  基于Shape-Matching的刚体模拟

- bunny初始位置[0, 0.6, 0]，按 ‘l’ 发射，按 'r' 重置

  impulst-based方法结果：

![result](.\Lab1_Angry Bunny\result_impulse-based.gif)

​	shape-matching方法结果：


![result](\Lab1_Angry Bunny\result_shape-matching.gif)

### Lab2：Cloth Simulation（满分）

- Implicit Cloth Solver \ Position-Based Dynamics (PBD)

- 拖动小球和布料发生碰撞

  Implicit方法结果：

![result](.\Lab2_Cloth Simulation\result_implicit.gif)

​		PBD方法结果：

![result](\Lab2_Cloth Simulation\result_PBD.gif)

### Lab3：Elasitc Body Simulation（满分）

- Finite Element Method (FEM)，considering StVK \ neo-Hookean model

- Reference：

  [Huamin Wang and Yin Yang 2016. Descent Methods for Elastic Body Simulation on the GPU. *ACM Transactions on Graphics* (SIGGRAPH Asia)](https://web.cse.ohio-state.edu/~wang.3602/Wang-2016-DME/Wang-2016-DME.pdf)

  [CUDA code for GPU-based hyperelastic simulation in C++](https://web.cse.ohio-state.edu/~wang.3602/Wang-2016-DME/Wang-2016-DME.zip)

FVM方法

![result](\Lab3_Bouncy House\result_FVM.gif)

StVK模型：

![result](\Lab3_Bouncy House\result_StVK.gif)

neo-Hookean方法：

![result](\Lab3_Bouncy House\result_neo-Hookean.gif)

### Lab4：Ripples Simulation（满分）

- Shallow_Wave model \ Two-way coupling
- 按 ‘r' 添加随机水滴，拖动方块进行交互

![result](\Lab4_Pool Ripples\result.gif)

