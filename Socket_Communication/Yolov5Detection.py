import yolov5
import torch
import cv2
import numpy as np
import urllib
import zed_client
import pickle


class ObjDetModel:

    def __init__(self, model_name):
        # load pretrained model
        self.model = yolov5.load(model_name)

    def load_new_model(self, model_name):
        self.model = yolov5.load(model_name)

    def detect_in_image(self, image):
        frame_cc = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Run the YOLO model
        results = self.model(frame_cc, 320)
        return results
        
    def draw_boxes(self, image, results):
        for box in results.xyxy[0]:
            if box[5] == 0:
                xB = int(box[2])
                xA = int(box[0])
                yB = int(box[3])
                yA = int(box[1])
                cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
        return image
