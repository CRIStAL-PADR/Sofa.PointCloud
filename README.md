# Sofa.PointCloud

> A SOFA framework plugin for real-time rendering of 3D reconstructed objects using **Gaussian Splatting**.

---

## Overview

**Sofa.PointCloud** is a plugin for the [**SOFA Framework**](https://www.sofa-framework.org), dedicated to interactive physical simulation and visualization.  
It enables the loading, manipulation, and rendering of **3D point clouds** (e.g., from 3D reconstruction or scanning) using a **Gaussian Splatting** rendering approach — providing smooth and high-quality visualization of dense, unstructured point data.

This plugin integrates seamlessly into SOFA scenes and can be combined with other simulation or visualization components.

[![Watch the video](https://github.com/user-attachments/assets/b966ab18-ca10-4230-a582-1a4e9e329772)](https://youtube.com/playlist?list=PLXhlzEXHKONM6hGw3cx-sIgVKAdVQFc4D&si=5uyvOKv-PZtP_D23)


---

## Key Features

- Load point cloud files (`.ply`, files)
- Real-time rendering using **Gaussian Splatting**:
- Integrated with the **SOFA rendering pipeline** and scenes. 
- Compatible with existing SOFA camera and lighting systems

---

## Dependencies & Submodules

- **SOFA Framework** at master
- **OpenGL / GLSL** ≥ 4.3 (for Gaussian Splat shaders)
- **Eigen** (for geometric and linear algebra operations) 
- **(submodule) Liteviz-gz** (OpenGL renderering)
- **(submodule) graphdeco-differentiable-renderer** (Experimental CUDA rendering)

---

## Installation
### 0. Sofa Version
The plugin is in developement so it should compile with the master version of SOFA. 
Tested on Sofa.Qt runSOFA version as well as runSOFAGFLW.
If you use SofaGFLWImGUI, you need to merge this bugfix: https://github.com/sofa-framework/SofaGLFW/pull/245

### 1. Clone the repository
```bash
git clone --recurse-submodules https://github.com/<user>/Sofa.PointCloud.git
cd Sofa.PointCloud
```

If you already cloned the repository without submodules, initialize them manually:
```bash
git submodule update --init --recursive
```

### 2. Compilation
Add the plugin in your existing compilation workflow and compile it. 

## Execution
Add the SOFA_DATA_PATH=PATH_TO_THE_ASSETS_DIRECTORY so the shaders and .ply files are found. 
Go to the example and run them. 


