#opencv import
import cv2
#socket import
import socket
#import for packing
import sys
import pickle
import struct
import pyzed.sl as sl


class Client:
    def __init__(self, host, port):
        self.HOST = host
        self.PORT = port

    def connect_to_server(self):
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("Connecting to the server...")
            self.client_socket.connect((self.HOST, self.PORT))
            print(f"Connected to the server at {self.HOST}")
        except Exception as e:
            print(f"Connection error: {e}")
            self.client_socket = None

    def send_video(self, zed_frame):
        if self.client_socket is None:
            print("Connection not established. Cannot send video.")
            return

        print("Video feed set")

        try:
            while True:
                _, frame = zed_frame
                small_frame = cv2.resize(frame, None, fx=0.4, fy=0.4)
                data = pickle.dumps(small_frame)
                self.client_socket.sendall(struct.pack("=L", len(data)) + data)
        except KeyboardInterrupt:
            print("Video streaming stopped.")
        except Exception as e:
            print(f"Error sending video data: {e}")
        finally:
            self.client_socket.close()

def main():
    host = '127.0.0.1'
    port = 8089
    client = Client(host, port)
    client.connect_to_server()
    print("zed camera 1")
    # Create a ZED camera object
    zed = sl.Camera()
    print("zed camera 2")
    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    print("zed camera 3")
    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(-1)
    print("zed camera 4")
        # Set the input from stream
    init = sl.InitParameters()
    
    # init.set_from_stream(host, port) # Specify the IP and port of the sender

    print("Initialized zed camera")
    while True:
        # create image objesct
        image_zed = sl.Mat()

        # Zed image object exists
        if zed.grab() == sl.ERROR_CODE.SUCCESS:

            # image types can be changed below VIEW.(TYPE)
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)

            # numpy data array for converting zed to open-cv
            image_ocv = image_zed.get_data()

            client.send_video(image_ocv)

            

    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()