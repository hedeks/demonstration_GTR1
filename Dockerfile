FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

# Non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install system dependencies and C++ libraries
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    pkg-config \
    libopencv-dev \
    liboctomap-dev \
    libeigen3-dev \
    libzmq3-dev \
    libprotobuf-dev \
    protobuf-compiler \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python requirements
# Install PyTorch with CUDA 11.8 support
RUN pip3 install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
# Install other dependencies
RUN pip3 install --no-cache-dir pyzmq protobuf opencv-python ultralytics numpy websockets

# Set working directory
WORKDIR /app

# Copy the project files (assuming code is mounted or copied)
COPY . /app

# Generate Python Protobuf bindings
RUN protoc -I=02_data_models -I=04_interfaces --python_out=src/ml_service 02_data_models/gtr1_structures.proto 04_interfaces/ipc_messages.proto

# Build C++ Core inside Docker so it starts instantly
RUN mkdir -p build && cd build && cmake .. && make -j4

# Make the launch script executable
RUN chmod +x run_demo.sh

# Expose ports for Dashboard HTTP (8000) and WebSockets (8080)
EXPOSE 8000
EXPOSE 8080

CMD ["./run_demo.sh"]
