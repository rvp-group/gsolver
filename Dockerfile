# Use Ubuntu 22.04 as base image
FROM ubuntu:22.04

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    pkg-config \
    libyaml-cpp-dev \
    libomp-dev \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    libgtk-3-dev \
    libxkbcommon-x11-0 \
    vulkan-tools \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglib2.0-0 \
    libxext6 \
    libsm6 \
    libxrender1 \
    libfontconfig1 \
    libice6 \
    libx11-6 \
    libx11-xcb1 \
    libxcb1 \
    libxcb-dri2-0 \
    libxcb-dri3-0 \
    libxcb-present0 \
    libxcb-sync1 \
    libxshmfence1 \
    libxxf86vm1 \
    libdrm2 \
    x11-utils \
    # TeX dependencies for matplotlib usetex
    texlive-latex-recommended \
    texlive-latex-extra \
    texlive-fonts-recommended \
    # Additional font packages to provide type1ec.sty and improved font support
    cm-super \
    texlive-fonts-extra \
    dvipng \
    ghostscript \
    && rm -rf /var/lib/apt/lists/*

# Create a Python virtual environment
RUN python3 -m venv /opt/venv

# Activate virtual environment and install Python packages
ENV PATH="/opt/venv/bin:$PATH"
RUN pip install --upgrade pip
# Required Python packages for the examples/scripts.
RUN pip install rerun-sdk pyyaml numpy matplotlib

# Set working directory
WORKDIR /workspace

# Create a convenient build command for gsolver
RUN echo '#!/bin/bash\nset -e\ncd /workspace\nmkdir -p build\ncd build\ncmake .. -DGSOLVER_BUILD_EXAMPLES=ON -DGSOLVER_BUILD_PYTHON=ON\nmake -j"$(nproc)"\n' > /usr/local/bin/build-target && \
    chmod +x /usr/local/bin/build-target

# Set environment variables to ensure Python virtual environment is active
ENV PATH="/opt/venv/bin:$PATH"
ENV VIRTUAL_ENV="/opt/venv"

# Default command
CMD ["/bin/bash"]
