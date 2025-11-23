ARG PYTORCH="2.1.0"
ARG CUDA="11.8"
ARG CUDNN="8"
ARG MMCV2="2.1.0"

FROM pytorch/pytorch:${PYTORCH}-cuda${CUDA}-cudnn${CUDNN}-runtime

ENV TORCH_CUDA_ARCH_LIST="6.0 6.1 7.0+PTX"
ENV TORCH_NVCC_FLAGS="-Xfatbin -compress-all"
ENV CMAKE_PREFIX_PATH="$(dirname $(which conda))/../"
ENV MMCV=${MMCV2}

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
        git ninja-build g++ cmake \
        libsm6 libxext6 libglib2.0-0 libxrender-dev libgl1-mesa-dev \
        wget vim \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN conda clean --all -f

# PyTorch 2.1.0 modules were compiled against NumPy 1.x and crash with NumPy 2.x.
RUN pip uninstall -y numpy && \
    pip install numpy==1.26.4
RUN pip install openmim 

# MMEngine for MMCV2/MMSeg1.2
RUN mim install "mmengine>=0.10.0"

# Install MMCV 2.x (prebuilt wheel exists for CUDA 11.8 + PT 2.1)
RUN mim install "mmcv==2.1.0"

# Install MMSegmentation
RUN git clone -b main https://github.com/open-mmlab/mmsegmentation.git /mmsegmentation
WORKDIR /mmsegmentation

RUN pip install -r requirements.txt
RUN pip install -e .
RUN pip install numpy==1.26.4

# Download data
RUN wget http://dags.stanford.edu/data/iccv09Data.tar.gz -O stanford_background.tar.gz
RUN tar xf stanford_background.tar.gz
RUN rm stanford_background.tar.gz

# Prepare data
COPY prepare_data.py .
RUN python3 prepare_data.py

# Copy scripts
COPY see_example.py .
COPY see_example_mask.py .
COPY train.py .

