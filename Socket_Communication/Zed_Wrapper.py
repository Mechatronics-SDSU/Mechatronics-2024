import pyzed.sl as sl
import cv2
import copy

class Zed:
    
    def __init__(self):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 60

    def open(self):
        state = self.zed.open(self.init_params)
        return state

    def get_cv2_thing(self):
        _, frame = self.cap.read()
        return frame

    def get_image(self):
        image = None
        image_zed = sl.Mat()
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(image_zed, sl.VIEW.RIGHT)
            return copy.deepcopy(image_zed.get_data())


    def get_depth_image(self):
        image_zed = sl.Mat()
        if (self.zed.grab() == sl.ERROR_CODE.SUCCESS):
            self.zed.retrieve_image(image_zed, sl.VIEW.DEPTH)
            return copy.deepcopy(image_zed.get_data())
        
    def get_min_depth(self, x1, y1, x2, y2):
        depth_zed = sl.Mat(self.zed.get_camera_information().camera_resolution.width, 
                           self.zed.get_camera_information().camera_resolution.height, 
                           sl.MAT_TYPE.F32_C1)
        
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS :
            # Retrieve depth data (32-bit)
            self.zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
            # Load depth data into a numpy array
            depth_ocv = depth_zed.get_data()
            # Print the depth value at the center of the image
            print(depth_ocv[int(len(depth_ocv)/2)][int(len(depth_ocv[0])/2)])
        

        
def main():
    z = Zed()
    while True:
        image = z.get_image()
        cv2.imshow("image_test", image)
        cv2.waitKey(1)
        continue

if __name__ == '__main__':
    main()

