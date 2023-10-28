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
    
    def draw_lines(self, image, results):
        # Draw a line from the center of the image to the center of the detected object
        start = (int(image.shape[1] / 2), int(image.shape[0] / 2))
        end = None
        for box in results.xyxy[0]:
                if box[5] == 0:
                    xB = int(box[2])
                    xA = int(box[0])
                    yB = int(box[3])
                    yA = int(box[1])
                    end = ((xB + xA) // 2, (yB + yA) // 2)
        if end is not None:
            cv2.line(image, start, end, (255, 255, 255), 5)
        return image
        
