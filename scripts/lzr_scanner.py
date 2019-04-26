"""This file contains the driver for the lzr safety scanner"""

import serial
import serial.tools.list_ports
import serial.rs485

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import binascii


class LzrScanner():
    """the driver is specially for the LZR U920"""

    def __init__(self, name, baudrate):
        self.ser = None

        self.name = name
        self.baudrate = baudrate                            # data transmission speed

        # preset values
        self.sync = [b'\xff', b'\xfe', b'\xfd', b'\xfc']    # indicate the start of a frame

        # contents
        self.message = None
        self.msg_size = None
        self.data = None

        self.cmd = None

        self.plane_size = 274                               # number of points in each plane

        self.p2_enable = True                               # switch of P2
        self.p2_msg = None                                  # raw data of P2 (bytes)
        self.p2_dists = None                                # metric data of P2 (integers)
        self.p2_points = None                               # 3-d position of P2

        self.p4_enable = True
        self.p4_msg = None
        self.p4_dists = None
        self.p4_points = None

        self.p1_enable = True
        self.p1_msg = None
        self.p1_dists = None
        self.p1_points = None

        self.p3_enable = True
        self.p3_msg = None
        self.p3_dists = None
        self.p3_points = None

        # angle matrix computation to convert dist info to 3-d position info
        self.start_pos = -48.0 / 180.0 * np.pi              # degree [-48, 48] in radian
        self.step_size = 0.3516 / 180.0 * np.pi             # angular resolution in radian
        self.thetas = np.asarray([(self.start_pos + i * self.step_size)
                                  for i in range(self.plane_size)]).astype('float32')
                                                            # list of angles to be computed

        self.plane_angle = 2.0 / 180.0 * np.pi              # angle between the adjacent planes
        self.alphas = np.asarray([i * self.plane_angle
                                for i in range(4)]).astype('float32')

        self.converter = np.ones((self.plane_size, 3))
        self.converter[:, 0] = np.cos(self.thetas)
        self.converter[:, 1] = np.sin(self.thetas)

    def connect(self):
        """
        Establish the connection to the scanner using the rs485 interface.
        According to the communication protocol, we use one stop bit and no
        parity bit.
        """
        self.ser = serial.rs485.RS485(self.name,
                                      self.baudrate,
                                      stopbits=serial.STOPBITS_ONE,
                                      parity=serial.PARITY_NONE)

    def capture_frame(self):
        """search for one frame and store the frame info"""

        # search for the sync bytes which indicate the start of one frame
        # sync_bytes = [None, None, None, None]
        # while True:
        #     sync_bytes[3] = sync_bytes[2]
        #     sync_bytes[2] = sync_bytes[1]
        #     sync_bytes[1] = sync_bytes[0]
        #     sync_bytes[0] = binascii.hexlify(self.ser.read())
        #
        #     # check the content
        #     try:
        #         if (sync_bytes[0] + sync_bytes[1] + sync_bytes[2]
        #             + sync_bytes[3] == b'fffefdfc'):
        #             print("Frame captured!")
        #             break
        #     except TypeError:
        #         pass

        while not self.lookup_sync():
            pass

        # print('Frame captured!')
        self.msg_size = int.from_bytes(self.ser.read(2),
                                       byteorder='little',
                                       signed=True)
        # print('Msg Size: {}'.format(self.msg_size))

        # raw message info (cmd + options + data)
        self.message = self.ser.read(self.msg_size)

        # command info
        self.cmd = int.from_bytes(self.message[:2], byteorder='little', signed=False)

        # raw data info (plane_num + distance values)
        self.p3_msg = self.message[-549:]
        self.p1_msg = self.message[-1098: -549]
        self.p4_msg = self.message[-1647: -1098]
        self.p2_msg = self.message[-2196: -1647]
        # print(len(self.p3_msg), len(self.p1_msg), len(self.p4_msg), len(self.p2_msg))

        # examine the msg size
        try:
            assert (self.p3_msg[0], self.p1_msg[0], self.p4_msg[0], self.p2_msg[0]) \
                   == (3, 2, 1, 0), "Fail to interpret the msg"
        except AssertionError:
            # print("error\n\n")
            return -1

        # convert bytes to integers (ignore the plane_num)
        self.p3_dists = [int.from_bytes([self.p3_msg[2 * i + 1], self.p3_msg[2 * i + 2]],
                                        byteorder='little', signed=True)
                         for i in range((len(self.p3_msg) - 1) // 2)]
        self.p1_dists = [int.from_bytes([self.p1_msg[2 * i + 1], self.p1_msg[2 * i + 2]],
                                        byteorder='little', signed=True)
                         for i in range((len(self.p1_msg) - 1) // 2)]
        self.p4_dists = [int.from_bytes([self.p4_msg[2 * i + 1], self.p4_msg[2 * i + 2]],
                                        byteorder='little', signed=True)
                         for i in range((len(self.p4_msg) - 1) // 2)]
        self.p2_dists = [int.from_bytes([self.p2_msg[2 * i + 1], self.p2_msg[2 * i + 2]],
                                        byteorder='little', signed=True)
                         for i in range((len(self.p2_msg) - 1) // 2)]

        # convert list into np array for further processing
        self.p3_dists = np.asarray(self.p3_dists).astype('float32').reshape(274, 1)
        self.p1_dists = np.asarray(self.p1_dists).astype('float32').reshape(274, 1)
        self.p4_dists = np.asarray(self.p4_dists).astype('float32').reshape(274, 1)
        self.p2_dists = np.asarray(self.p2_dists).astype('float32').reshape(274, 1)

        # print(self.p3_dists[132:142])
        # print(self.p1_dists[132:142])
        # print(self.p4_dists[132:142])
        # print(self.p2_dists[132:142])

        # Compute the position info
        # print(self.converter)
        # print(self.thetas.shape)
        self.p3_points = self.converter * np.array([[np.cos(self.alphas[2]), np.cos(self.alphas[2]),
                                                     np.sin(self.alphas[2])]], dtype='float32') * self.p3_dists
        self.p1_points = self.converter * np.array([[np.cos(self.alphas[0]), np.cos(self.alphas[0]),
                                                     np.sin(self.alphas[0])]], dtype='float32') * self.p1_dists
        self.p4_points = self.converter * np.array([[np.cos(self.alphas[3]), np.cos(self.alphas[3]),
                                                     np.sin(self.alphas[3])]], dtype='float32') * self.p4_dists
        self.p2_points = self.converter * np.array([[np.cos(self.alphas[1]), np.cos(self.alphas[1]),
                                                     np.sin(self.alphas[1])]], dtype='float32') * self.p2_dists
        # print(self.p1_points[132:142])

        return 0

    def lookup_sync(self, flag=0):
        """recursion to find the sync bytes"""
        if flag == 1 or self.ser.read() == self.sync[3]:
            if self.ser.read() == self.sync[2]:
                if self.ser.read() == self.sync[1]:
                    if self.ser.read() == self.sync[0]:
                        return True
                    elif self.ser.read() == self.sync[-1]:
                        return self.lookup_sync(flag=1)
                    else:
                        return False
                elif self.ser.read() == self.sync[-1]:
                    return self.lookup_sync(flag=1)
                else:
                    return False
            elif self.ser.read() == self.sync[-1]:
                return self.lookup_sync(flag=1)
            else:
                return False
        else:
            return False

    def visualize_scan(self):
        """visualize the 3d points"""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(self.p1_points[:, 0], self.p1_points[:, 1], self.p1_points[:, 2], c='r')
        ax.scatter(self.p2_points[:, 0], self.p2_points[:, 1], self.p2_points[:, 2], c='g')
        ax.scatter(self.p3_points[:, 0], self.p3_points[:, 1], self.p3_points[:, 2], c='b')
        ax.scatter(self.p4_points[:, 0], self.p4_points[:, 1], self.p4_points[:, 2])

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        plt.show()
