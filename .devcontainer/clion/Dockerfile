ARG VERSION=latest

FROM ghcr.io/ethz-asl/wavemap_ros1:${VERSION}

# Install dependencies
# hadolint ignore=DL3008
RUN apt-get update && apt-get install -yq --no-install-recommends curl sudo && rm -rf /var/lib/apt/lists/*

# Install CLion in the container
ARG CLION_VERSION=2023.2
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# hadolint ignore=DL3003
RUN mkdir -p /opt/clion && \
    curl -L "https://download.jetbrains.com/cpp/CLion-${CLION_VERSION}.tar.gz" \
    | tar -C /opt/clion --strip-components 1 -xzvf - && \
    cd /opt/clion/bin && ./remote-dev-server.sh registerBackendLocationForGateway

# Create a non-root user
ARG USERNAME=ci
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID $USERNAME && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Make CLion available to the current user
USER $USERNAME
# hadolint ignore=DL3003
RUN cd /opt/clion/bin && ./remote-dev-server.sh registerBackendLocationForGateway

# Prebuild the package
RUN catkin build wavemap_all

# Make root the default user
# NOTE: This is currently required for CLion's local deployment procedure to work.
# hadolint ignore=DL3002
USER root
