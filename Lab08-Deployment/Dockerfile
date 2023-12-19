FROM nvcr.io/nvidia/tensorrt:22.12-py3

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
	apt-get install -y \
	ffmpeg libsm6 libxext6 libcanberra-gtk-module libcanberra-gtk3-module libopencv-dev

# Create the workspace
RUN mkdir -p /workspace/

# ONNX
WORKDIR /workspace
RUN git clone https://github.com/MichalNowicki/ONNX-YOLOv8-Object-Detection.git
WORKDIR /workspace/ONNX-YOLOv8-Object-Detection
RUN pip install -r requirements.txt


# TensorRT
WORKDIR /workspace
RUN git clone https://github.com/triple-Mu/YOLOv8-TensorRT.git
