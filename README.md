# Layered Wavemap
<a href="https://github.com/GuilhermeCabaco/wavemap-digital-twin/actions/workflows/cpp.yml"><img src="https://img.shields.io/github/actions/workflow/status/ethz-asl/wavemap/cpp.yml?label=C%2b%2b&logo=C%2b%2b&logoColor=white" alt="C++"/></a>
<a href="https://github.com/GuilhermeCabaco/wavemap-digital-twin/actions/workflows/python.yml"><img src="https://img.shields.io/github/actions/workflow/status/ethz-asl/wavemap/python.yml?label=Python&logo=python&logoColor=white" alt="Python"/></a>
<a href="https://github.com/GuilhermeCabaco/wavemap-digital-twin/actions/workflows/ros1.yml"><img src="https://img.shields.io/github/actions/workflow/status/ethz-asl/wavemap/ros1.yml?label=ROS1&logo=ros&logoColor=white" alt="ROS1"/></a>
<a href="https://github.com/GuilhermeCabaco/wavemap-digital-twin/actions/workflows/docs.yml"><img src="https://img.shields.io/github/actions/workflow/status/ethz-asl/wavemap/docs.yml?label=Docs&logo=sphinx&logoColor=white" alt="Docs"/></a>
<a href="https://github.com/ethz-asl/wavemap/releases"><img src="https://img.shields.io/github/v/tag/ethz-asl/wavemap?label=Version&logo=semver" alt="Version"/></a>
<a href="https://github.com/GuilhermeCabaco/wavemap-digital-twin/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-BSD%203-blue?logo=bsd" alt="License"/></a>

Layered Wavemap extends Wavemap's compressed multi-resolution occupancy map with typed environmental attributes. Continuous fields such as reflectivity, signal, near-infrared intensity, and RGB share the wavelet hierarchy with occupancy. Categorical fields such as semantic classes use aligned sparse side layers with lossless majority-with-exceptions compression.

[Get started](docs/pages/tutorials/layered_wavemap.rst) · [Run the RGB demo](#try-the-layered-rgb-demo) · [Original Wavemap documentation](https://ethz-asl.github.io/wavemap/)

[![Layered Wavemap demo](https://i.ytimg.com/vi/l24QWAxm7Zw/hqdefault.jpg)](https://youtu.be/l24QWAxm7Zw)

*Watch the Layered Wavemap demo on YouTube.*

## Try the layered RGB demo

The synthetic forest demo needs no sensor data. On Ubuntu 20.04 with ROS Noetic:

```bash
cd ~/catkin_ws
catkin build wavemap_all
source devel/setup.bash
roslaunch wavemap_ros synthetic_rgb_layered_map.launch
```

The launch file opens RViz and progressively builds a colored layered map. Docker setup, native installation, real Ouster usage, map persistence, and a layer-extension walkthrough are in the [Layered Wavemap guide](docs/pages/tutorials/layered_wavemap.rst).

## Wavemap foundation
Wavemap achieves state-of-the-art memory and computational efficiency by combining Haar wavelet compression and a coarse-to-fine measurement integration scheme. Advanced measurement models allow it to attain exceptionally high recall rates on challenging obstacles like thin objects.

The framework is very flexible and supports several data structures, measurement integration methods, and sensor models out of the box. The ROS interface can, for example, easily be configured to fuse multiple sensor inputs, such as a LiDAR configured with a range of 20m and several depth cameras up to a resolution of 1cm, into a single multi-resolution occupancy grid map.

Wavemap provides [C++](https://ethz-asl.github.io/wavemap/pages/tutorials/cpp) and [Python](https://ethz-asl.github.io/wavemap/pages/tutorials/python) APIs and an interface to [ROS1](https://ethz-asl.github.io/wavemap/pages/tutorials/ros1). The code is extensively tested on Intel, AMD and ARM CPUs on Ubuntu 20.04, 22.04 and 24.04. Example Docker files [are available](https://github.com/ethz-asl/wavemap/tree/main/tooling/docker) and documented in the [installation instructions](https://ethz-asl.github.io/wavemap/pages/installation/index). We [welcome contributions](https://ethz-asl.github.io/wavemap/pages/contributing).

⭐ If you find wavemap useful, star it on GitHub to get notified of new releases!


## Documentation
The framework's documentation is available on [GitHub Pages](https://ethz-asl.github.io/wavemap/) for easy online access. A PDF version of each release’s documentation can also be found in the respective [release notes](https://github.com/ethz-asl/wavemap/releases).

### Table of contents
* [Layered Wavemap: setup, use, and extension](docs/pages/tutorials/layered_wavemap.rst)
* [Installation](https://ethz-asl.github.io/wavemap/pages/installation)
* [Demos](https://ethz-asl.github.io/wavemap/pages/demos)
* [Tutorials](https://ethz-asl.github.io/wavemap/pages/tutorials)
* [Parameters](https://ethz-asl.github.io/wavemap/pages/parameters)
* [Contributing](https://ethz-asl.github.io/wavemap/pages/contributing)
* [C++ API](https://ethz-asl.github.io/wavemap/cpp_api/unabridged_api)
* [Python API](https://ethz-asl.github.io/wavemap/python_api)
* [FAQ](https://ethz-asl.github.io/wavemap/pages/faq)

## Paper
A technical introduction to the theory behind wavemap is provided in our open-access RSS paper, available [here](https://www.roboticsproceedings.org/rss19/p065.pdf). For a quick overview, watch the accompanying 5-minute presentation [here](https://www.youtube.com/live/ftQhK75Ri1E?si=9txTYyJ78wQuhyN-&t=733).

<details>
<summary>Abstract</summary>
<br>
Volumetric maps are widely used in robotics due to their desirable properties in applications such as path planning, exploration, and manipulation. Constant advances in mapping technologies are needed to keep up with the improvements in sensor technology, generating increasingly vast amounts of precise measurements. Handling this data in a computationally and memory-efficient manner is paramount to representing the environment at the desired scales and resolutions. In this work, we express the desirable properties of a volumetric mapping framework through the lens of multi-resolution analysis. This shows that wavelets are a natural foundation for hierarchical and multi-resolution volumetric mapping. Based on this insight we design an efficient mapping system that uses wavelet decomposition. The efficiency of the system enables the use of uncertainty-aware sensor models, improving the quality of the maps. Experiments on both synthetic and real-world data provide mapping accuracy and runtime performance comparisons with state-of-the-art methods on both RGB-D and 3D LiDAR data. The framework is open-sourced to allow the robotics community at large to explore this approach.
</details>

Please cite this paper when using wavemap for research.

APA-style:
```
Reijgwart, V., Cadena, C., Siegwart, R., & Ott, L. (2023). Efficient volumetric mapping of multi-scale environments using wavelet-based compression. Proceedings of Robotics: Science and Systems XIX. https://doi.org/10.15607/RSS.2023.XIX.065
```

BibTeX:
```
@INPROCEEDINGS{reijgwart2023wavemap,
    author = {Reijgwart, Victor and Cadena, Cesar and Siegwart, Roland and Ott, Lionel},
    journal = {Robotics: Science and Systems. Online Proceedings},
    title = {Efficient volumetric mapping of multi-scale environments using wavelet-based compression},
    year = {2023-07},
}
```

Note that the code has significantly improved since the paper was written. Wavemap is now up to 10x faster, thanks to new multi-threaded measurement integrators, and uses up to 50% less RAM, by virtue of new memory efficient data structures inspired by [OpenVDB](https://github.com/AcademySoftwareFoundation/openvdb).
