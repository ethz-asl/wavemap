ARG VERSION=latest

FROM ghcr.io/ethz-asl/wavemap_ros1:${VERSION}

# Build the package
RUN catkin build wavemap_all
