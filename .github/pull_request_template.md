# Description

Thank you for opening a PR! Please summarize the changes in 1–2 sentences.

## Type of change

- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that causes existing functionality to not work as expected)
- [ ] Other (please describe):

## Detailed Summary

Provide the motivation, context, and links to any related issues, PRs, or documentation:

- Motivation: Why is this change necessary?
- Context: How does it fit into wavemap's functionality?
- Related issues/PRs: Fixes # (issue) / Links to other PRs

## API Changes

List any changes to wavemap's APIs to help users update their code. Write "None" if there are no changes.

### C++ API:

*

### Python API:

*

### ROS1 Interface:

*

## Review Notes

Is there anything specific the reviewers should focus on, or are there unresolved questions? Mention them here.

# Testing

### Automated Tests

Have you added or modified unit tests to verify these changes? If not, let us know if you'd like assistance.

### Manual Tests

If manual tests were performed to verify these changes, describe them here and include instructions to reproduce them.
Describe test configurations where applicable.

**System information (optional):**

- CPU: [e.g., Intel i9-9900K]
- GPU: [e.g., Nvidia RTX 2080Ti]
- RAM: [e.g., 32GB]
- OS: [e.g., Ubuntu 20.04]
- API: [e.g., C++, Python, ROS1]
- Installation: [e.g., pre-built Docker, local CMake, Pip, catkin]

**Runtime information (optional):**

- Launch file: [e.g., Link or GitHub Gist]
- Config file: [e.g., Link or GitHub Gist]
- Dataset name (if public): [e.g., Newer College Cloister]
- Custom setup (for private datasets, or live usage):
    - Depth sensor: [e.g., Livox MID360 LiDAR]
    - Pose source: [e.g., Odometry from FastLIO2]

For performance or accuracy-related changes, include the above system and runtime information and describe:

- **Performance (optional)**
    - Measured operation: [e.g. serializing the map, performing 1M queries, processing dataset X]
    - Metrics [e.g., CPU time, wall time, total RAM usage]
- **Accuracy (optional)**
    - Metrics: [e.g., AUC, accuracy, recall]
- **Summary of changes**
    - What metrics improved and by how much?
    - Did any metrics worsen?

### Benchmarks (To be completed by maintainers)

We will rerun wavemap's benchmarks and report the results here to validate there are no general performance/accuracy regressions.

# Checklist

General

- [ ] My code follows the style guidelines of this project
- [ ] I have performed a self-review of my code
- [ ] I have commented my code, particularly in hard-to-understand areas
- [ ] I have added or updated tests as required
- [ ] Any required changes in dependencies have been committed and pushed

Documentation (where applicable)

- [ ] I have updated the installation instructions (in docs/pages/installation)
- [ ] I have updated the code's inline API documentation (e.g., docstrings)
- [ ] I have updated the parameter documentation (in docs/pages/parameters)
- [ ] I have updated/extended the tutorials (in docs/pages/tutorials)
