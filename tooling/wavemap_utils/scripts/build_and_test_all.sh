#!/bin/bash

set -o pipefail

catkin build wavemap_all
catkin test wavemap wavemap_3d
