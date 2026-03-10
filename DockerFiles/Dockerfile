FROM ubuntu:24.04 AS builder

# Install build essentials
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-pip \
    libgl1-mesa-dev \
    libx11-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libeigen3-dev \
    libomp-dev \
    xorg-dev \
    libxcb-shm0 \
    libglu1-mesa-dev \
    libc++-dev \
    libc++abi-dev \
    libsdl2-dev \
    libxi-dev \
    libtbb-dev \
    libosmesa6-dev \
    libudev-dev \
    autoconf \
    libtool \
    cmake \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/isl-org/Open3D && cd Open3D && mkdir build && cd build && cmake .. && make -j$(nproc)

RUN cd Open3D/build && make install DESTDIR=/open3d-install

# ============================================================
# CUDA BASE (Ubuntu 24.04 + GPU)
# ============================================================
FROM nvidia/cuda:13.1.0-runtime-ubuntu24.04

COPY --from=builder /open3d-install/ /

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_INDIRECT=0

# ============================================================
# BASE UBUNTU PACKAGES
# ============================================================
RUN apt-get update --allow-releaseinfo-change && \
    apt-get upgrade -y && \
    apt-get install -y \
    apt-transport-https \
    ca-certificates \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    git \
    wget \
    sudo \
    nano \
    tmux \
    xterm \
    iputils-ping \
    iproute2 \
    mesa-utils \
    libgl1 \
    libglu1-mesa \
    libc++1 \
    libc++abi1 \
    libtbb12 \
    libomp5 \
    python3-pip \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Open3D in /usr/local may reference LLVM-18 libc++ absolute paths.
# Create compatibility symlinks so CMake/Make can resolve them.
RUN mkdir -p /usr/lib/llvm-18/lib && \
    ln -sf /usr/lib/x86_64-linux-gnu/libc++.so.1 /usr/lib/llvm-18/lib/libc++.so.1 && \
    ln -sf /usr/lib/x86_64-linux-gnu/libc++abi.so.1 /usr/lib/llvm-18/lib/libc++abi.so

# Open3D installs a private TBB in /usr/local/lib that conflicts with the
# distro-provided /usr/lib/x86_64-linux-gnu/libtbb.so.12 and causes CMake RPATH warnings.
# Replace Open3D's TBB with symlinks to system TBB so Open3D's CMake config works
# but uses the system library.
RUN rm -f /usr/local/lib/libtbb.so* && \
    ln -sf /usr/lib/x86_64-linux-gnu/libtbb.so.12.11 /usr/local/lib/libtbb.so.12.12 && \
    ln -sf /usr/lib/x86_64-linux-gnu/libtbb.so.12 /usr/local/lib/libtbb.so.12 && \
    ln -sf /usr/lib/x86_64-linux-gnu/libtbb.so /usr/local/lib/libtbb.so

# ============================================================
# ROS2 REPOSITORY
# ============================================================
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
> /etc/apt/sources.list.d/ros2.list

# ============================================================
# ROS2 INSTALL
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-plotjuggler-ros \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ============================================================
# PYTHON ML DEPENDENCIES
# ============================================================
RUN apt-get update && apt-get remove -y \
    python3-sympy \
    python3-mpmath \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages --ignore-installed pip setuptools wheel

# PyTorch with CUDA support (cu118) - optional, only needed for YOLO detector
# If build fails due to network issues, comment out this RUN block
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
    --retries 10 \
    --timeout 300 \
    torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/cu118 || \
    echo "WARNING: PyTorch installation failed. YOLO detector will not work, but ICP calibration will."

# Computer Vision and scientific stack from apt (more robust in restricted SSL environments)
RUN apt-get update && apt-get install -y \
    python3-opencv \
    python3-scipy \
    python3-sklearn \
    python3-matplotlib \
    python3-pil \
    python3-yaml \
    python3-tqdm \
    python3-requests \
    && rm -rf /var/lib/apt/lists/*

# Remaining PyPI packages
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
    --trusted-host pypi.org \
    --trusted-host files.pythonhosted.org \
    --trusted-host pypi.python.org \
    "numpy<2" \
    ultralytics

# ============================================================
# USER
# ============================================================
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd -g $USER_GID $USERNAME 2>/dev/null || groupadd $USERNAME && \
    useradd -m -u $USER_UID -g $USERNAME -s /bin/bash $USERNAME 2>/dev/null || \
    useradd -m -g $USERNAME -s /bin/bash $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ============================================================
# WORKSPACE (created as root, then switch user)
# ============================================================
RUN mkdir -p /home/$USERNAME/ros2_ws/src && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME && \
    echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "if [ -f install/setup.bash ]; then source install/setup.bash; fi" >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws
