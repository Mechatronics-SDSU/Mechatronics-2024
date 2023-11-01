import pyzed.sl as sl
import math
import cv2
import numpy as np
import copy
import statistics

class Zed:
    
    def __init__(self):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 60
        #self.init_params.depth_mode = sl.DEPTH_MODE.NEURAL

    def open(self):
        state = self.zed.open(self.init_params)
        return state

    def get_cv2_thing(self):
        _, frame = self.cap.read()
        return frame

    def get_image(self):
        image_zed = sl.Mat()
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(image_zed, sl.VIEW.RIGHT)
            return copy.deepcopy(image_zed.get_data())


    def get_depth_image(self):
        image_zed = sl.Mat()
        if (self.zed.grab() == sl.ERROR_CODE.SUCCESS):
            self.zed.retrieve_image(image_zed, sl.VIEW.DEPTH)
            image = image_zed.get_data()
            return copy.deepcopy(image)
        
    def get_median_depth(self, x1, y1, x2, y2):
        width = self.zed.get_camera_information().camera_resolution.width
        height = self.zed.get_camera_information().camera_resolution.height

        if x1 > width or x2 > width:
            return -1
        elif y1 > height or y2 > height:
            return -1
        elif self.zed.grab() != sl.ERROR_CODE.SUCCESS :
            return -1
        
        depth_zed = sl.Mat(self.zed.get_camera_information().camera_resolution.width, 
                            self.zed.get_camera_information().camera_resolution.height, 
                            sl.MAT_TYPE.F32_C1)
        
        # Retrieve depth data (32-bit)
        self.zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
        # Print the depth value at the center of the image
        depth_list = list()
        skip_size = 5
        for x in range(x1, x2 - (skip_size), skip_size):
            for y in range(y1, y2 - (skip_size), skip_size):
                _, depth = depth_zed.get_value(x, y)
                if not math.isnan(depth) and math.isfinite(depth):
                    depth_list.append(int(depth))

        if not depth_list:
            return -1
        median = statistics.median_low(depth_list)
        #print(median)
        return median
    

        
def main():
    zed = Zed()
    state = zed.open()
    while True:
        image = zed.get_depth_image()
        if (image is not None):
            cv2.imshow("image_test", image)
            cv2.waitKey(1)
        continue

if __name__ == '__main__':
    main()

