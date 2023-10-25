import yolov5
import torch
import cv2
import numpy as np
import urllib
import zed_client
import pickle


class Yolov5Detection:

    def __init__(self, host, port):
        # load pretrained model
        self.model = yolov5.load('yolov5m.pt')
        self.client_obj = zed_client.Client(host, port)
        self.client_obj.connect_to_server()

    #default to localhost
    def __init__(self):
        # load pretrained model
        self.model = yolov5.load('yolov5m.pt')
        self.client_obj = zed_client.Client('127.0.0.1', 8089)
        self.client_obj.connect_to_server()

    def detect_in_image(self, image, with_boxes):
        frame_cc = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Run the YOLO model
        results = self.model(frame_cc, 320)

        if not with_boxes:
            self.client_obj.send_video(image)
        else:
            for box in results.xyxy[0]:
                if box[5] == 0:
                    xB = int(box[2])
                    xA = int(box[0])
                    yB = int(box[3])
                    yA = int(box[1])
                    cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
            self.client_obj.send_video(image)

    def close_socket(self):
        self.client_obj.client_socket.close()

def main():
    yolo = Yolov5Detection()
    cap = cv2.VideoCapture(0)
    while True:
        _, frame = cap.read()
        yolo.detect_in_image(frame, True)

if __name__ == '__main__':
    main()
