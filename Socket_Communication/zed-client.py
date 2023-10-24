import cv2
import socket
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

    def send_video_feed(self):
        if self.client_socket is None:
            print("Connection not established. Cannot send video feed.")
            return

        try:
            zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD720
            init_params.camera_fps = 60

            if zed.open(init_params) == sl.ERROR_CODE.SUCCESS:
                print("Initialized ZED camera")

                while True:
                    image_zed = sl.Mat()
                    if zed.grab() == sl.ERROR_CODE.SUCCESS:
                        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
                        image_ocv = image_zed.get_data()
                        small_frame = cv2.resize(image_ocv, None, fx=0.4, fy=0.4)
                        data = pickle.dumps(small_frame)
                        self.client_socket.sendall(struct.pack("=L", len(data)) + data)
            else:
                print("Failed to open ZED camera.")
        except KeyboardInterrupt:
            print("Video feed stopped.")
        except Exception as e:
            print(f"Error sending video feed: {e}")
        finally:
            self.client_socket.close()
            zed.close()

def main():
    host = '127.0.0.1'
    port = 8089
    client = Client(host, port)
    client.connect_to_server()
    client.send_video_feed()
    

if __name__ == "__main__":
    main()