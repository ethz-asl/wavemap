#!/bin/bash

set -o pipefail

catkin build wavemap_all
catkin test wavemap_common wavemap_2d wavemap_3d
