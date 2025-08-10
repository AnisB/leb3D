This repository contains a project that implements the technique described in the paper [**Adaptive Tetrahedral Grids for Volumetric Path-Tracing**](https://arxiv.org/abs/2506.11510) published at the conference *Siggraph 2025*.

This demo relies on **DX12** and some **Shader model 6.6** features. Here are the steps to get it running:

- Clone the repo and open a terminal in the root folder of the repo
- Run the script **dependencies.bat**
- Create a build folder or **mkdir build**
- Move to that folder or **cd build**
- Open a terminal if it is not already the case and type **cmake ..**

This will generate a solution that you can open in Microsoft Visual Studio and the two executables. One that converts a grid structure to a leb3D structure and one that allows to visualize both.

Cheers!
