#opencv import
import cv2
#socket import
import socket
#import for packing
import sys
import pickle
import struct



class Client:
    def __init__(self, host, port):
        self.HOST = host
        self.PORT = port
        self.client_socket = None
        print("Set Client Variables")

    def connect_to_server(self):
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("Connecting to the server...")
            self.client_socket.connect((self.HOST, self.PORT))
            print(f"Connected to the server at {self.HOST}")
        except Exception as e:
            print(f"Connection error: {e}")
            self.client_socket = None

    def send_video(self, frame):
        
        if self.client_socket is None:
            print("Connection not established. Cannot send video.")
            return

        try:
            small_frame = cv2.resize(frame, None, fx=0.4, fy=0.4)
            data = pickle.dumps(small_frame)
            self.client_socket.sendall(struct.pack("=L", len(data)) + data)
        except Exception as e:
            raise Exception(f"Error occurred while sending video data: {e}")