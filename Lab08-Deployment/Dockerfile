FROM ultralytics/ultralytics:8.3.77

RUN pip3 install tensorrt
RUN pip3 install cuda-python==12.4.0 --force-reinstall

# TensorRT for YOLO
WORKDIR /ultralytics
RUN git clone https://github.com/Linaom1214/TensorRT-For-YOLO-Series.git

# coco128 for calibration
RUN wget https://ultralytics.com/assets/coco128.zip && unzip coco128 && rm coco128.zip

# Example image for inference
RUN wget https://raw.githubusercontent.com/ultralytics/ultralytics/refs/heads/main/ultralytics/assets/bus.jpg