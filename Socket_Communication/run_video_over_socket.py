import Yolov5Detection as yv5
import zed_client
import argparse
import cv2
import pyzed.sl as sl

parser = argparse.ArgumentParser()
parser.add_argument('-host_ip', help='ip to send images to', required=False)
parser.add_argument('-port', help='port to send images over', required=False)
parser.add_argument('-show_boxes',help='boolean to show object detection boxes', required=False)
parser.add_argument('-model_name', help='model to run on', required=True)
args = parser.parse_args()

def main():
    host = args.host_ip
    port = args.port
    show_boxes = args.show_boxes
    model_name = args.model_name
    print(model_name)

    if host is None:
        host = '127.0.0.1'

    if port is None:
        port = 8089

    if show_boxes is None:
        show_boxes = True

    if model_name is None:
        model_name = 'yolov5m.pt'

    socket = zed_client.Client(host, port)
    socket.connect_to_server()
    detection = yv5.ObjDetModel(model_name)

    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Comes in HD1080 , HD2K, HD720, VGA, LAST
    init_params.camera_fps = 60  # FPS of 15, 30, 60, or 100
    camera_state = zed.open(init_params)


    while True:
        image_zed = sl.Mat()

        # Zed image object exists
        if zed.grab() == sl.ERROR_CODE.SUCCESS:

            # image types can be changed below VIEW.(TYPE)
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)

            # numpy data array for converting zed to open-cv
            image_ocv = image_zed.get_data()

            detection.detect_in_image(image_ocv, show_boxes)
            socket.send_video(image_ocv)


if __name__ == '__main__':
    main()