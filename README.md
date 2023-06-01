# Wavemap
Wavemap is a hierarchical, multi-resolution occupancy mapping framework. By employing Haar wavelet compression and a hierarchical measurement integration scheme, it achieves both high memory and computational efficiency. Among others, these efficiency improvements make the use of demanding uncertainty-aware sensor models tractable and allow wavemap to achieve exceptional recall rates on challenging obstacles such as thin objects.

The framework is very flexible and supports several data structures, integration schemes, measurement models and projection models out of the box. Inputs can be provided both as point clouds or depth images. Furthermore, any number of inputs with potentially different settings and weights can simultaneously be fused into a single map.

We test the code both on Intel and ARM. At the moment, only ROS1 is supported, but we would be interested in adding ROS2 support. Please [reach out to us](https://github.com/ethz-asl/wavemap/issues) if you are interested in collaborating.

## Paper
When using wavemap for research, please cite the following paper [[preprint](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/614632/RSS22_WavemapFinalPreprintCompressed.pdf?sequence=1&isAllowed=y)]:


```
@INPROCEEDINGS{reijgwart2023wavemap,
    author = {Reijgwart, Victor and Cadena, Cesar and Siegwart, Roland and Ott, Lionel},
    journal = {Robotics: Science and Systems. Online Proceedings},
    title = {Efficient volumetric mapping of multi-scale environments using wavelet-based compression},
    year = {2023-07},
}
```

<details>
<summary>Abstract</summary>
<br>
Volumetric maps are widely used in robotics due to their desirable properties in applications such as path planning, exploration, and manipulation. Constant advances in mapping technologies are needed to keep up with the improvements in sensor technology, generating increasingly vast amounts of precise measurements. Handling this data in a computationally and memory-efficient manner is paramount to representing the environment at the desired scales and resolutions. In this work, we express the desirable properties of a volumetric mapping framework through the lens of multi-resolution analysis. This shows that wavelets are a natural foundation for hierarchical and multi-resolution volumetric mapping. Based on this insight we design an efficient mapping system that uses wavelet decomposition. The efficiency of the system enables the use of uncertainty-aware sensor models, improving the quality of the maps. Experiments on both synthetic and real-world data provide mapping accuracy and runtime performance comparisons with state-of-the-art methods on both RGB-D and 3D LiDAR data. The framework is open-sourced to allow the robotics community at large to explore this approach.
</details>

Note that the code has significantly been improved since the paper was written. In terms of performance, wavemap now includes multi-threaded measurement integrators and faster, more efficient data structures inspired by [OpenVDB](https://github.com/AcademySoftwareFoundation/openvdb).

## Install
To use wavemap with ROS, we recommend using ROS Noetic as installed following the [standard instructions](http://wiki.ros.org/noetic/Installation). Other ROS1 distributions should also work, but have not yet been tested.

Start by installing the necessary system dependencies

```shell script
sudo apt update
sudo apt install git python3-catkin-tools python3-vcstool -y
```

Create a catkin workspace with

```shell script
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/
source /opt/ros/noetic/setup.sh
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Clone the code for wavemap and its catkin dependencies

```shell script
# With SSH keys
cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/voxgraph.git
vcs import --recursive --input wavemap/tooling/vcstool/wavemap_ssh.yml .
```

<details>
<summary>No ssh keys?</summary>
<br>

```shell
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/voxgraph.git
vcs import --recursive --input wavemap/tooling/vcstool/wavemap_https.yml .
```

</details>

Then install the remaining system dependencies using

```shell script
cd ~/catkin_ws/src
rosdep update
rosdep install --ignore-src --skip-keys="numpy_eigen catkin_boost_python_buildtool" -y
```

Build wavemap's ROS interface and the Rviz plugin used to visualize its maps with

```shell script
cd ~/catkin_ws/
catkin build wavemap_3d_ros wavemap_rviz_plugin
```

## Run
#### Demo
*Instructions coming soon.*

#### Your own dataset
The basic requirements for running wavemap are:
1. an odometry source, and
2. a source of dense depth or 3D LiDAR data data.

*More instructions coming soon*

# Contributors
