# GAMES103_Assignments

repo for the assignments of GAMES103: 基于物理的计算机动画入门 by 王华民

这个项目记录了[GAMES103课程](http://games-cn.org/games103/)的PPT和作业



### 项目框架

> GAMES103课程PPT
>
> >  [01_intro.pptx](GAMES103课程PPT\01_intro.pptx)(with no viedos)
> >
> >  [02_math.pptx](GAMES103课程PPT\02_math.pptx) 
> >
> >  [03_rigid.pptx](GAMES103课程PPT\03_rigid.pptx) 
> >
> >  [04_rigid contact.pptx](GAMES103课程PPT\04_rigid contact.pptx) 
> >
> >  [05_cloth.pptx](GAMES103课程PPT\05_cloth.pptx) 
> >
> >  [06_constraints.pptx](GAMES103课程PPT\06_constraints.pptx) 
> >
> >  [07_collision.pptx](GAMES103课程PPT\07_collision.pptx) 
> >
> >  [08_FEM.pptx](GAMES103课程PPT\08_FEM.pptx) 
> >
> >  [08_FEM_new.pptx](GAMES103课程PPT\08_FEM_new.pptx) 
> >
> >  [09_FEM2.pptx](GAMES103课程PPT\09_FEM2.pptx) 
> >
> >  [10_wave.pptx](GAMES103课程PPT\10_wave.pptx) 
> >
> >  [11_Eulerian_fluids.pptx](GAMES103课程PPT\11_Eulerian_fluids.pptx) 
> >
> >  [12_SPH.pptx](GAMES103课程PPT\12_SPH.pptx) 
> >
> > [After-class reading list.docx](GAMES103课程PPT\After-class reading list.docx)

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

![result](\Lab1：Angry Bunny\result_impulse-based.gif)

​		shape-matching方法结果：

![result](\Lab1：Angry Bunny\result_shape-matching.gif)

### Lab2：Cloth Simulation（满分）

- Implicit Cloth Solver \ Position-Based Dynamics (PBD)
- 拖动小球和布料发生碰撞

![result](\Lab2：Cloth Simulation\result_implicit.gif)

![result](\Lab2：Cloth Simulation\result_PBD.gif)

### Lab3：Elasitc Body Simulation（满分）

- Finite Element Method (FEM)，considering StVK \ neo-Hookean model

- Reference：

  [Huamin Wang and Yin Yang 2016. Descent Methods for Elastic Body Simulation on the GPU. *ACM Transactions on Graphics* (SIGGRAPH Asia)](https://web.cse.ohio-state.edu/~wang.3602/Wang-2016-DME/Wang-2016-DME.pdf)

  [CUDA code for GPU-based hyperelastic simulation in C++](https://web.cse.ohio-state.edu/~wang.3602/Wang-2016-DME/Wang-2016-DME.zip)

![result](\Lab3：Bouncy House\result_FVM.gif)

![result](\Lab3：Bouncy House\result_StVK.gif)

![result](\Lab3：Bouncy House\result_neo-Hookean.gif)

### Lab4：Ripples Simulation（满分）

- Shallow_Wave model \ Two-way coupling
- 按 ‘r' 添加随机水滴，拖动方块进行交互

![result](\Lab4：Pool Ripples\result.gif)

