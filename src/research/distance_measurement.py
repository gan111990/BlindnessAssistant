import signal
from rplidar import RPLidar

PORT_NAME = "/dev/ttyUSB0"
RANGE = 5

lidar = RPLidar(PORT_NAME)
lidar.connect()
# lidar.start()

try:
    i = 0
    for scan in lidar.iter_measures():
        print(scan)
        new_scan = scan[0]
        quality = scan[1]
        angle = round(scan[2], 2)
        distance = round(scan[3] / 10, 2)

        if ((angle <= RANGE / 2) or (angle >= 360 - RANGE / 2)):
            # if (distance == 0.0):
            #     print("Sample (#{0}) is in range, but couldn't measure distance".format(i))
            # else:
            #     print("Sample (#{0}): {1} cm at {2}° with {3}/15 quality".format(i, distance, angle, quality))
            if distance != 0.0:
                print("Sample (#{0}): {1} cm at {2}° with {3}/15 quality".format(i, distance, angle, quality))
        i += 1
except KeyboardInterrupt:
    lidar.stop()
    lidar.disconnect()
    exit(1)

