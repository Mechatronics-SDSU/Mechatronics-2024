import socket
import cv2
import pickle
import struct

class Server:
    def __init__(self, host, port, package_size):
        self.HOST = host
        self.PORT = port
        self.PACKAGE_SIZE = package_size

    def set_socket(self):
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print('Socket created')

            server_socket.bind((self.HOST, self.PORT))
            print('Socket bind complete')
            server_socket.listen(10)
            print('Socket now listening')

            client_socket, client_address = server_socket.accept()
            print("Accepted Client Connection")
            data = b''
            payload_size = struct.calcsize("=L")
            return server_socket, client_socket, data, payload_size
        except Exception as e:
            print(f"Socket setup error: {e}")
            return None, None, None, None

    def receive_and_display_images(self):
        server_socket, client_socket, data, payload_size = self.set_socket()
        print("Socket Connection Completed")
        cv2.startWindowThread()
        cv2.namedWindow("Image")
        print("Created window")

        while True:
            while len(data) < payload_size:
                data += client_socket.recv(4096)
            packed_msg_size = data[:payload_size]

            data = data[payload_size:]
            msg_size = struct.unpack("=L", packed_msg_size)[0]

            while len(data) < msg_size:
                data += client_socket.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame = pickle.loads(frame_data)
            cv2.imshow("Image", frame)
            cv2.waitKey(3)

def main():
    host = '146.244.98.37'
    port = 8089
    package_size = 442504
    server = Server(host, port, package_size)
    server.receive_and_display_images()

if __name__ == '__main__':
    main()