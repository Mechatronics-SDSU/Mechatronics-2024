import Yolov5Detection as yv5
import Socket_Client
import argparse
import math
import cv2
import gui_helper
import multiprocessing
import time

import_success = True

try:
    import pyzed.sl as sl
    from Zed_Wrapper import Zed
except:
    print("Zed library not found")
    import_success = False

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

def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
    except AttributeError:
        print('special key {0} pressed'.format(
            key))


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

def create_camera_object():
    zed = None
    cap = None
    if import_success:
        zed = Zed()
        state = zed.open()
        if state != sl.ERROR_CODE.SUCCESS:
            zed = None
            print("Zed camera not found, using webcam")
            cap = cv2.VideoCapture(0)
    else:
        zed = None
        print("camera library not found, using webcam")
        cap = cv2.VideoCapture(0)

    return zed, cap


def send_image_to_socket(socket, image):
    try:
        socket.send_video(image)
    except Exception as e:
        print(e)
        socket.client_socket.close()

def create_gui_processes(image, results):
    box_process = multiprocessing.Process(target = gui_helper.draw_boxes(image, results))
    line_process = multiprocessing.Process(target = gui_helper.draw_lines(image, results))
    return box_process, line_process

def get_image(zed, cap):
    if zed is not None:
        image = zed.get_image()
    elif cap is not None:
        image = get_image_from_webcam(cap)
    else:
        print("No camera found, exiting")
    
    return image


def main():
    host, port, show_boxes, model_name, get_depth, show_depth = parse_arguments()

    socket = Socket_Client.Client(host, port)
    socket.connect_to_server()
    detection = yv5.ObjDetModel(model_name)
    depth = 0

    zed, cap = create_camera_object()

    image = get_image(zed, cap)
    results = detection.detect_in_image(image)

    test_start = time.time()
    not_swapped = True

    while True:
        start_time = time.perf_counter()
        image = get_image(zed, cap)

        #testing hot swapping models
        if time.time() - test_start> 10 and not_swapped:
            print("swap")
            detection.load_new_model('./models_folder/best.pt')
            not_swapped = False

        #run yolo detection
        results = detection.detect_in_image(image)

        #get depth image from the zed if zed is initialized and user added the show depth argument
        if zed is not None and show_depth:
            image = zed.get_depth_image()
        #if show 
        if show_boxes:
            image = gui_helper.draw_boxes(image, results)
            image = gui_helper.draw_lines(image, results)

        if zed is not None and get_depth:
            depth = get_nearest_object(results, zed)
            #print("depth: ", depth)
        
        #starting imu code
        orientation, lin_acc, ang_vel = zed.get_imu()
        #print("orientation : \t", orientation)
        #print("linear acceleration: \t", lin_acc)
        #print("angular velocity: \t", ang_vel)


        #send image over socket on another processor
        send_process = multiprocessing.Process(target = send_image_to_socket(socket, image))
        send_process.start()
        end_time = time.perf_counter()
        #print("loop time: ", end_time - start_time)
        


if __name__ == '__main__':
    main()