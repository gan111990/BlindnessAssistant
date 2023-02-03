from threading import Thread
from pyrplidar import PyRPlidar
from math import sin, cos, radians, floor
from collections import OrderedDict
import copy
import sys
import os.path
import numpy as np
find_src = lambda p: os.path.sep.join(p[:p.index('src') + 1])
SRC_PATH = find_src(os.path.realpath(__file__).split(os.path.sep))
if SRC_PATH in sys.path:
    pass
else:
    sys.path.insert(0, SRC_PATH)


class LidarStream:
    def __init__(self, port_name='/dev/ttyUSB0'):
        self.port_name = port_name
        self.lidar = None
        self.generator = None
        # Initialize the object distance and x, y co-ordinates
        self.scans = OrderedDict()
        # initialize the variable used to indicate if the thread should be stopped
        self.stopped = False

    def start(self):
        # initialize the lidar for the stream
        self.lidar = PyRPlidar()
        self.lidar.connect(port=self.port_name)
        self.lidar.set_motor_pwm(500)
        self.generator = self.lidar.start_scan()
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    # @timeit
    def update(self):
        # keep looping infinitely until the thread is stopped
        temp_scans = OrderedDict()
        for i, scan in enumerate(self.generator()):
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                self.lidar.stop()
                self.lidar.set_motor_pwm(0)
                self.lidar.disconnect()
                self.lidar = None
                return
            # otherwise, scan
            angle = floor(scan.angle)
            if angle == 359:
                self.scans = copy.deepcopy(temp_scans)
                temp_scans = OrderedDict()

            distance = round(scan.distance / 1000, 2)
            # Calculate the x and y coordinates of the point
            x = distance * np.cos(np.deg2rad(scan.angle))
            y = distance * np.sin(np.deg2rad(scan.angle))
            temp_scans[angle] = (x, y, distance)

    def read(self):
        # return the lidar full scans
        return self.scans

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True


if __name__ == "__main__":
    import pprint
    lidarStream = LidarStream()
    lidarStream.start()
    print('Start while loop')
    try:
        while True:
            scans = lidarStream.read()
            print(pprint.pformat(scans))
    except KeyboardInterrupt:
        lidarStream.stop()
    except InterruptedError:
        lidarStream.stop()
    print('Finish while loop')