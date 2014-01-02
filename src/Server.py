#!/usr/bin/env python
import threading
import socket
import time
import rospy
from robot.msg import controller
import struct

class ControllerPub():
    def __init__(self):
        self.control_pub = rospy.Publisher('controller', controller)
    def pub(self, message):
        if (len(message) == 4):
            data = struct.unpack('=4B', message)
            msg = controller(left_duty_cycle=data[0], left_direction=data[1],
                    right_duty_cycle=data[2], right_direction=data[3])
            self.control_pub.publish(msg)
        else: print "Message of length != 4 received and discarded"
        return

class ClientThread(threading.Thread):
    def __init__(self, client_socket):
        threading.Thread.__init__(self)
        self.client = client_socket

    def run(self):
        control = ControllerPub()
        done = False;
        command = self.client.recv(1024)
        global shutdown
        while not done:
            if command == 'quit':
                done = True;
            elif command is not None:
                print list(command)
                control.pub(command)
            command = self.client.recv(1024)
        self.client.close()
        return

class Server():
    def __init__(self):
        self.socket = None;
    def run(self):
        initialized = False;
        while not initialized:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket.bind(('', 6000))
                self.socket.listen(1)
                initialized = True;
            except socket.error:
                print 'Socket could not open'
                del self.socket
                time.sleep(1)
        try:
            while True:
                client = self.socket.accept()[0]
                client_thread = ClientThread(client)
                print 'Opened client thread'
                client_thread.daemon = True;
                client_thread.start()
        except KeyboardInterrupt:
            print 'Keyboard interrupt. Closing server'
        self.socket.close()

if __name__ == "__main__":
    rospy.init_node('Server')
    server = Server()
    server.run()
    print "Closed"
