import Yolov5Detection as yv5
import Socket_Client
import argparse
import math
import cv2
import copy
from Zed_Wrapper import Zed

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
parser.add_argument('-show_depth', help='show depth map', required=False)
parser.add_argument('-get_depth', help='get depth map', required=False)
args = parser.parse_args()

def get_image_from_webcam(cap):
    ret, frame = cap.read()
    return frame

def get_nearest_object(results, zed):
    nearest_object = math.inf
    for box in results.xyxy[0]:
        if box[5] != 0:
            continue

        median = zed.get_median_depth(int(box[0]), int(box[1]), int(box[2]), int(box[3]))
        if not math.isnan(median) and not math.isinf(median) and not median <= 0:
            nearest_object = min(median, nearest_object)
            
    return nearest_object


def parse_arguments():
    host = args.host_ip
    port = args.port
    show_boxes = args.show_boxes
    model_name = args.model_name
    get_depth = args.get_depth
    show_depth = args.show_depth

    if host is None:
        host = '127.0.0.1'

    if port is None:
        port = 8089

    if show_boxes is None or show_boxes == 'True':
        show_boxes = True
    else:
        show_boxes = False

    if get_depth is None or get_depth == 'True':
        get_depth = True
    else:
        get_depth = False

    if show_depth is None or show_depth == 'False':
        show_depth = False
    else:
        show_depth = True

    if model_name is None:
        model_name = './models_folder/yolov5m.pt'
    else:
        model_name = "./models_folder/" + model_name

    return host, port, show_boxes, model_name, get_depth, show_depth

def main():
    host, port, show_boxes, model_name, get_depth, show_depth = parse_arguments()

    socket = Socket_Client.Client(host, port)
    socket.connect_to_server()
    detection = yv5.ObjDetModel(model_name)

    zed = None
    cap = None
    state = None
    depth = 0

    zed = Zed()
    state = zed.open()

    if state != sl.ERROR_CODE.SUCCESS:
        zed = None
        print("Zed camera not found, using webcam")
        cap = cv2.VideoCapture(0)

    while True:
        if zed is not None:
            image = zed.get_image()
        elif cap is not None:
            image = get_image_from_webcam(cap)
        else:
            print("No camera found, exiting")
            break

        results = detection.detect_in_image(image)

        if zed is not None and get_depth:
            depth = get_nearest_object(results, zed)
            print("depth: ", depth)
        if show_depth:
            image = zed.get_depth_image()
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