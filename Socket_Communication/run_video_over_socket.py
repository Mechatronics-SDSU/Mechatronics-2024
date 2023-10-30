import Yolov5Detection as yv5
import zed_client
import argparse
import cv2

TARGET_SIZE = 640

try:
    import pyzed.sl as sl
except:
    print("Zed library not found")

parser = argparse.ArgumentParser()
parser.add_argument('-host_ip', help='ip to send images to', required=False)
parser.add_argument('-port', help='port to send images over', required=False)
parser.add_argument('-show_boxes',help='boolean to show object detection boxes', required=False)
parser.add_argument('-model_name', help='model to run on', required=False)
args = parser.parse_args()

def get_image_from_webcam(cap):
    ret, frame = cap.read()
    return frame

def main():
    host = args.host_ip
    port = args.port
    show_boxes = args.show_boxes
    model_name = args.model_name

    if host is None:
        host = '127.0.0.1'

    if port is None:
        port = 8089

    if show_boxes is None or show_boxes == 'True':
        show_boxes = True
    else:
        show_boxes = False

    if model_name is None:
        model_name = './models_folder/yolov5m.pt'
    else:
        model_name = "./models_folder/" + model_name

    socket = zed_client.Client(host, port)
    socket.connect_to_server()
    detection = yv5.ObjDetModel(model_name)

    zed = None
    cap = None
    state = None

    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Comes in HD1080 , HD2K, HD720, VGA, LAST
    init_params.camera_fps = 60  # FPS of 15, 30, 60, or 100
    state = zed.open(init_params)

    if state != sl.ERROR_CODE.SUCCESS:
        zed = None
        print("Zed camera not found, using webcam")
        cap = cv2.VideoCapture(0)


    while True:
        image = None
        if zed is not None:
            image_zed = sl.Mat()
            if zed.grab() == sl.ERROR_CODE.SUCCESS:   
                zed.retrieve_image(image_zed, sl.VIEW.LEFT)
                image = image_zed.get_data()

        elif cap is not None:
            image = get_image_from_webcam(cap)
        else:
            print("No camera found, exiting")
            break

        results = detection.detect_in_image(image)
        if show_boxes:
            detection.draw_boxes(image, results)
            detection.draw_lines(image, results)
        
        try:
            socket.send_video(image)
        except Exception as e:
            print(e)
            socket.client_socket.close()
            break
        


if __name__ == '__main__':
    main()