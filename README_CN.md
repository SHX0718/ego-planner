这是一个关于 EGO-Planner 和 EGO-Swarm 项目的 README 文档翻译。

---

# 注意！

我们最近开发的规划器 [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 是 EGO-Planner 的升级版本。
它更加鲁棒和安全，因此我们更推荐使用它。
如果你只有一架无人机，只需将 EGO-Swarm 启动文件（launch files）中的 `drone_id` 设置为 `0` 即可。
当然，一些话题（topic）名称与 EGO-Planner 有所不同，请使用 `rqt_graph` 和 `rosnode info <package name>` 进行检查。

# ROS2 支持

关于 ROS2 版本，请参考 ego-planner-swarm 仓库的 [ros2_version](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version) 分支。

# 3分钟快速上手

代码已在安装了 ROS 的 Ubuntu **16.04, 18.04 和 20.04** 上通过编译测试。
你可以直接逐行执行以下命令：

```bash
sudo apt-get install libarmadillo-dev
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
cd ego-planner
catkin_make
source devel/setup.bash
roslaunch ego_planner simple_run.launch

```

如果你连接 Github 的网络较慢，我们建议你尝试 Gitee 仓库 [https://gitee.com/iszhouxin/ego-planner](https://gitee.com/iszhouxin/ego-planner)。它们会自动同步。

如果你觉得这项工作有用或有趣，请给我们一颗星 :star:，谢谢！:grinning:

# 致谢 (Acknowledgements)

* 本仓库的框架基于 Zhou Boyu 的 [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)，他在四旋翼局部规划方面取得了令人印象深刻的性能。
* 我们使用的 L-BFGS 求解器来自 [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite)。
它是一个仅包含头文件的 C++ 单文件，轻量且易用。
* 仿真中生成的地图来自 William Wu 的 [mockamap](https://github.com/HKUST-Aerial-Robotics/mockamap)。
* 硬件架构基于 [Teach-Repeat-Replan](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan) 的开源实现。

# EGO-Planner

EGO-Planner: 一种用于四旋翼飞行器的无 ESDF（欧几里得符号距离场）基于梯度的局部规划器。

**EGO-Planner** 是一种轻量级的基于梯度的局部规划器，无需构建 ESDF，与一些最先进的方法相比 显著减少了计算时间。总规划时间仅需 **1ms 左右**，且无需计算 ESDF。

<p align = "center">
<img src="pictures/title.gif" width = "413" height = "232" border="5" />
<img src="pictures/comp.jpg" width = "413" height = "232" border="5" />
<img src="pictures/indoor.gif" width = "413" height = "232" border="5" />
<img src="pictures/outdoor.gif" width = "413" height = "232" border="5" />
</p>

**视频链接：** [YouTube](https://youtu.be/UKoaGW7t7Dk), [bilibili](https://www.bilibili.com/video/BV1VC4y1t7F4/) (中国大陆)

## 1. 相关论文

EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors, Xin Zhou, Zhepei Wang, Chao Xu and Fei Gao (被 RA-L 接收)。 [arXiv 预印本](https://arxiv.org/abs/2008.08835), [IEEE Xplore](https://ieeexplore.ieee.org/abstract/document/9309347), 以及 [IEEE Spectrum 报道](https://spectrum.ieee.org/automaton/robotics/robotics-hardware/video-friday-mit-media-lab-tf8-bionic-ankle)。

## 2. 标准编译

**要求**：安装了 ros-desktop-full 的 ubuntu 16.04, 18.04 或 20.04。

**步骤 1**. 安装 [Armadillo](http://arma.sourceforge.net/)，这是 **uav_simulator** 所必需的。

```
sudo apt-get install libarmadillo-dev

```

**步骤 2**. 从 Github 或 Gitee 克隆代码。这两个仓库会自动同步。

从 github：

```
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git

```

或从 gitee：

```
git clone https://gitee.com/iszhouxin/ego-planner.git

```

**步骤 3**. 编译：

```
cd ego-planner
catkin_make -DCMAKE_BUILD_TYPE=Release

```

**步骤 4**. 运行。

在 *ego-planner/* 文件夹下的终端中，打开 rviz 进行可视化和交互：

```
source devel/setup.bash
roslaunch ego_planner rviz.launch

```

在 *ego-planner/* 下的另一个终端中，运行仿真规划器：

```
source devel/setup.bash
roslaunch ego_planner run_in_sim.launch

```

然后你可以参照下面的 gif 动图来控制无人机。

<p align = "center">
<img src="pictures/sim_demo.gif" width = "640" height = "438" border="5" />
</p>

## 3. 使用 IDE

我们推荐使用 [vscode](https://code.visualstudio.com/)，你克隆的代码中已包含项目文件，即 *.vscode* 文件夹。
该文件夹默认是**隐藏**的。
按照以下步骤配置 IDE 以实现自动代码补全和跳转。
只需 3 分钟。

**步骤 1**. 在 vscode 中安装 C++ 和 CMake 扩展。

**步骤 2**. 使用以下命令重新编译代码：

```
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes

```

这将导出一个编译命令文件，帮助 vscode 确定代码架构。

**步骤 3**. 启动 vscode 并选择打开 *ego-planner* 文件夹。

```
code ~/<......>/ego-planner/

```

在 vscode 中按 **Ctrl+Shift+B** 编译代码。此命令定义在 *.vscode/tasks.json* 中。
你可以在 **"args"** 后添加自定义参数。默认为 **"-DCMAKE_BUILD_TYPE=Release"**。

**步骤 4**. 关闭并重启 vscode，你会看到 vscode 已经理解了代码架构，并能执行自动补全和跳转。

## 4. 是否使用 GPU

本仓库中的包 **local_sensing** 有 GPU 和 CPU 两个版本。为了更好的兼容性，默认设置为 CPU 版本。通过修改 **local_sensing** 包中 *CMakeList.txt* 的设置：

```
set(ENABLE_CUDA false)

```

改为：

```
set(ENABLE_CUDA true)

```

CUDA 将被开启以生成深度图像，模拟真实的深度相机。

请记得同时更改 *CMakeList.txt* 中以下部分的 'arch' 和 'code' 标志：

```
    set(CUDA_NVCC_FLAGS 
      -gencode arch=compute_61,code=sm_61;
    ) 

```

如果你因使用的 Nvidia 显卡不同而遇到编译错误，或者无法看到预期的正确深度图像，你可以通过 [链接1](https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/) 或 [链接2](https://github.com/tpruvot/ccminer/wiki/Compatibility) 检查正确的代码。

别忘了重新编译代码！

**local_sensing** 是模拟传感器模块。如果 `ENABLE_CUDA` 为 **true**，它会模拟立体相机测量的深度并通过 GPU 渲染深度图。如果 `ENABLE_CUDA` 为 **false**，它将发布不带光线投射的点云。我们的局部建图模块会自动选择深度图或点云作为其输入。

关于 CUDA 的安装，请访问 [CUDA ToolKit](https://developer.nvidia.com/cuda-toolkit)

## 5. 发挥 CPU 的全部性能

我们规划器的计算时间太短，操作系统来不及提升 CPU 频率，这导致计算时间往往变长且不稳定。

因此，我们建议你手动将 CPU 频率设置为最大值。
首先，安装工具：

```
sudo apt install cpufrequtils

```

然后你可以通过以下命令将 CPU 频率设置为允许的最大值：

```
sudo cpufreq-set -g performance

```

更多信息可以在这里找到：[http://www.thinkwiki.org/wiki/How_to_use_cpufrequtils](http://www.thinkwiki.org/wiki/How_to_use_cpufrequtils)。

注意，在高负载导致高温的情况下，CPU 频率可能仍会降低。

# 改进的 ROS-RealSense 驱动

我们修改了 ros-realsense 驱动，使激光发射器隔帧闪烁，从而在发射器的帮助下输出高质量的深度图像，同时获得不受激光干扰的双目图像。

<p align = "center">
<img src="pictures/realsense.PNG" width = "640" height = "158" border="5" />
</p>

此 ros 驱动修改自 [https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) 且兼容 librealsense2 2.30.0。
测试是在 Intel RealSense D435 和 D435i 上进行的。

参数 `emitter_on_off` 用于开启/关闭此附加功能。
注意，如果开启此功能，设备的输出帧率将减半，因为设备将一半的流用于深度估计，另一半作为双目灰度输出。
此外，在此设置下，参数 `depth_fps` 和 `infra_fps` 必须相同，且 `enable_emitter` 也必须为 true。

## 安装

必须明确安装 librealsense2 2.30.0 版本的驱动。
在 x86 CPU 上，这可以在 5 分钟内轻松完成。
首先，移除当前安装的驱动：

```
sudo apt remove librealsense2-utils

```

或者如果你是源码安装的 librealsense，请手动删除相关文件。
然后，你可以通过以下命令安装 2.30.0 版本的库：

```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

```

对于 ubuntu 16.04：

```
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u

```

对于 ubuntu 18.04：

```
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u

```

接着继续执行：

```
sudo apt-get install librealsense2-dkms
sudo apt install librealsense2=2.30.0-0~realsense0.1693
sudo apt install librealsense2-gl=2.30.0-0~realsense0.1693
sudo apt install librealsense2-utils=2.30.0-0~realsense0.1693
sudo apt install librealsense2-dev=2.30.0-0~realsense0.1693
sudo apt remove librealsense2-udev-rules
sudo apt install librealsense2-udev-rules=2.30.0-0~realsense0.1693

```

此时你可以通过以下命令验证安装：

```
realsense-viewer

```

## 运行

如果一切正常，你现在可以使用 `catkin_make` 编译名为 *modified_realsense2_camera.zip* 的 ros-realsense 包，然后运行 ros realsense 节点：

```
roslaunch realsense_camera rs_camera.launch

```

默认情况下，你将同时收到 30Hz 的深度流和双目流。

# 许可证 (Licence)

源代码在 [GPLv3](http://www.gnu.org/licenses/) 许可下发布。

# 维护 (Maintaince)

我们仍在致力于扩展该系统并提高代码的可靠性。

如有任何技术问题，请联系 Xin Zhou (iszhouxin@zju.edu.cn) 或 Fei GAO (fgaoaa@zju.edu.cn)。

如有商业咨询，请联系 Fei GAO (fgaoaa@zju.edu.cn)。