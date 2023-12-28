# Wavemap
<div>
<a href="https://github.com/ethz-asl/wavemap/actions/workflows/ci.yml"><img src="https://img.shields.io/github/actions/workflow/status/ethz-asl/wavemap/ci.yml?label=test&logo=githubactions&logoColor=white" alt="test"/></a>
<a href="https://github.com/ethz-asl/wavemap/actions/workflows/cd.yml"><img src="https://img.shields.io/github/actions/workflow/status/ethz-asl/wavemap/cd.yml?label=deploy&logo=githubactions&logoColor=white" alt="deploy"/></a>
<a href="https://ethz-asl.github.io/wavemap/"><img src="https://img.shields.io/badge/docs-online-brightgreen?logo=sphinx" alt="docs"/></a>
<a href="https://github.com/ethz-asl/wavemap/releases"><img src="https://img.shields.io/github/v/tag/ethz-asl/wavemap?label=release&logo=github" alt="release"/></a>
<a href="https://github.com/ethz-asl/wavemap/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-BSD%203-blue?logo=bsd" alt="license"/></a>
<a href="https://ethz-asl.github.io/wavemap/pages/contributing.html"><img src="https://img.shields.io/badge/contributions-welcome-brightgreen" alt="contributions welcome"/></a>
</div>
<div>
<a href="https://ethz-asl.github.io/wavemap/pages/installation.html"><img src="https://img.shields.io/badge/Intel-0071C5?logo=intel" alt="Intel"/></a>
<a href="https://ethz-asl.github.io/wavemap/pages/installation.html"><img src="https://img.shields.io/badge/AMD-ED1C24?logo=amd" alt="AMD"/></a>
<a href="https://ethz-asl.github.io/wavemap/pages/installation.html"><img src="https://img.shields.io/badge/Arm-0091BD?logo=arm&logoColor=white" alt="Arm"/></a>
<a href="https://github.com/ethz-asl/wavemap/pkgs/container/wavemap"><img src="https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=black" alt="docker"/></a>
</div>

[![3D reconstruction of Newer College's Cloister](https://github.com/ethz-asl/wavemap/assets/6238939/e432d4ea-440d-4e9d-adf9-af3ae3b09a10)](https://www.youtube.com/live/ftQhK75Ri1E?si=9txTYyJ78wQuhyN-&t=733)

## Hierarchical, multi-resolution volumetric mapping

Wavemap achieves state-of-the-art memory and computational efficiency by combining Haar wavelet compression and a coarse-to-fine measurement integration scheme. Advanced measurement models allow it to attain exceptionally high recall rates on challenging obstacles like thin objects.

The framework is very flexible and supports several data structures, measurement integration methods, and sensor models out of the box. The ROS interface can, for example, easily be configured to fuse multiple sensor inputs, such as a LiDAR configured with a range of 20m and several depth cameras up to a resolution of 1cm, into a single map.

⭐ If you find wavemap useful, star it on GitHub to get notified of new releases!

## Documentation
The framework's documentation is hosted on [GitHub Pages](https://ethz-asl.github.io/wavemap/).

### Table of contents
* [Installation](https://ethz-asl.github.io/wavemap/pages/installation)
* [Demos](https://ethz-asl.github.io/wavemap/pages/demos)
* [Configuration](https://ethz-asl.github.io/wavemap/pages/configuration)
* [Usage examples](https://ethz-asl.github.io/wavemap/pages/usage_examples)
* [Contributing](https://ethz-asl.github.io/wavemap/pages/contributing)
* [Library API](https://ethz-asl.github.io/wavemap/api/unabridged_api)
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
