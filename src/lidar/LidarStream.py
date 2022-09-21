from threading import Thread
from rplidar import RPLidar
from math import sin, cos, radians, floor
from collections import OrderedDict
import copy


class LidarStream:
    def __init__(self, port_name='/dev/ttyUSB0', detection_range=[80, 280]):
        # initialize the lidar for the stream
        self.detection_range = detection_range
        self.lidar = RPLidar(port_name)
        self.lidar.connect()
        # Initialize the object distance and x, y co-ordinates
        self.scans = OrderedDict()
        # initialize the variable used to indicate if the thread should be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        last_angle = 0
        last_acc = 0
        temp_scans = OrderedDict()
        for scan in self.lidar.iter_measures():
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                self.lidar.stop()
                self.lidar.disconnect()
                return
            # otherwise, scan
            angle = floor(angle)
            if last_angle == 359 and angle < last_angle:
                self.scans = copy.deepcopy(temp_scans)
                temp_scans = OrderedDict()

            if angle < self.detection_range[0]+1 or angle > self.detection_range[1]-1:
                if angle > last_angle or angle < 0:
                    last_angle = angle
                    last_acc = scan[1]
                distance = round(scan[3] / 10, 2)
                x = round(distance * cos(radians(scan[2])))
                y = round(distance * sin(radians(scan[2])))
                if last_angle == angle and scan[1] > last_acc:
                    temp_scans[angle] = {'x': x, 'y': y, 'distance': distance, 'accuracy': scan[1]}
                    last_acc = scan[1]

    def read(self):
        # return the lidar full scans
        return self.scans

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
