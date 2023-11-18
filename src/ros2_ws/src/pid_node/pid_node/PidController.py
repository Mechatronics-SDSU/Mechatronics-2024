from simple_pid import PID
import time
import numpy as np
# from can_client import Can_Client
import socket


pid_update_period = 1
is_running = False
pid_port = 10000
pid_host = "146.244.98.44"

yawP, yawI, yawD = .5, .1, .01
pitchP, pitchI, pitchD = .5, .1, .01
rollP, rollI, rollD = .5, .1, .01
xP, xI, xD = .5, .1, .01
yP, yI, yD = .5, .1, .01
zP, zI, zD = .5, .1, .01
# 
yawPID = PID(yawP, yawI, yawD, 0)
pitchPID = PID(pitchP, pitchI, pitchD, 0)
rollPID = PID(rollP, rollI, rollD, 0)
# 
xPID = PID(xP, xI, xD, 0)
yPID = PID(yP, yI, yD, 0)
zPID = PID(zP, zI, zD, 0)

controlMatrix = np.array([[0],  #yaw
                          [0],  #pitch
                          [0],  #roll
                          [0],  #x
                          [0],  #y
                          [0]]) #z

motorControlMatrix = np.array([[0],  #motor1 control value 
                               [0],  #motor2 control value 
                               [0],  #motor3 control value 
                               [0],  #motor4 control value 
                               [0],  #motor5 control value
                               [0],  #motor6 control value
                               [0],  #motor7 control value
                               [0]]) #motor8 control value

junebugMatrix = np.array([[1, 0, 0, 1, 0, 0],  #motor1 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0],  #motor2 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0],  #motor3 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0],  #motor4 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0],  #motor5 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0],  #motor6 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0],  #motor7 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0]]) #motor8 [yaw, pitch, roll, x, y, z]

class PidController():
    def __init__(self):
        # self.can_client = Can_Client()
        self.is_running = True

    def send_response(self):
        controlMatrix[0,0] = yawPID(-1) 
        controlMatrix[1,0] = pitchPID(-1)
        controlMatrix[2,0] = rollPID(-2)
        controlMatrix[3,0] = xPID(-3)
        controlMatrix[4,0] = yPID(-4)
        controlMatrix[5,0] = zPID(-5)

        motorControlMatrix = np.dot(junebugMatrix, controlMatrix)

        print("self.can_client.make_motor_request(self.convert_to_list(motorControlMatrix))")

    def convert_to_list(self, matrix):
        newlist = []

        for values in matrix:
            newlist.append(int([values][0]))
        
        return newlist

    def run_pids(self):
        self.server_socket, self.client_socket, self.data, self.payload_size = self.make_sockets()
        self.ThreadActive = True
        while self.is_running:
            self.send_response()
            time.sleep(pid_update_period)

    def stop_running(self):
        self.is_running = False

    def make_sockets(self):
        try: 
            client_socket, client_address = server_socket.accept()
            data = [0, 0, 0, 0, 0, 0]
            payload_size = struct.calcsize("=L")
            return server_socket, client_socket, data, payload_size
        except Exception as e:
            self.text_signal.emit("Socket Setup Error")
            return None, None, None, None
        except socket.error as err: 
            print ("socket creation failed with error %s" %(err))

    def get_current_axes(self):
        
    
def main(args = None):
    controller = PidController()
    try:
        controller.run_pids()
    except KeyboardInterrupt:
        print()
        print("shutting down PIDs...")
        controller.stop_running()

if __name__ == '__main__':
    main()