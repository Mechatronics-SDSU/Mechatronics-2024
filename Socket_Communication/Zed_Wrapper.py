import pyzed.sl as sl
import numpy as np
import cv2
import copy

class Zed:
    
    def __init__(self):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 60
        state = self.zed.open(self.init_params)
        print(state)

    def get_cv2_thing(self):
        _, frame = self.cap.read()
        return frame

    def get_image(self):
        image = None
        image_zed = sl.Mat()
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(image_zed, sl.VIEW.RIGHT)
            image = image_zed.get_data()
            return copy.deepcopy(image)


    def get_depth_image(self):
        image_zed = sl.Mat()
        if (self.zed.grab() == sl.ERROR_CODE.SUCCESS):
            self.zed.retrieve_image(image_zed, sl.VIEW.DEPTH)
            return image_zed.get_data()
        
def main():
    z = Zed()
    while True:
        image = z.get_image()
        cv2.imshow("image_test", image)
        cv2.waitKey(1)
        continue

if __name__ == '__main__':
    main()

