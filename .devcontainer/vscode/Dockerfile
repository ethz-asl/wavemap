ARG VERSION=latest

FROM ghcr.io/ethz-asl/wavemap_ros1:${VERSION}

# Create a non-root user and make it the default user
ARG USERNAME=ci
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID --shell /bin/bash $USERNAME && \
    cp -a /etc/skel/.bashrc /home/$USERNAME/ && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME

# Build the package
RUN catkin build wavemap_all
