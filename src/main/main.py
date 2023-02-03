import os
import sys

find_src = lambda p: os.path.sep.join(p[:p.index('src') + 1])
SRC_PATH = find_src(os.path.realpath(__file__).split(os.path.sep))
if SRC_PATH in sys.path:
    pass
else:
    sys.path.insert(0, SRC_PATH)

from threading import Thread
import numpy as np
import logging
# from lidar.LidarStream import LidarStream
from lidar.LidarStreamPylidar import LidarStream
from video.WebcamVideoStream import WebcamVideoStream
from util.util import measure_collision_distance, get_closest_objects


class Main:
    """
    Main thread class of Blindness Assistant
    """

    def __init__(self, inferer, config):
        self.config = config
        self.inferer = inferer
        self.video_getter = None
        self.lidar = None
        self.warning_msg = {"msg": "", "status": "read"}
        self.stopped = False

    def start(self):
        # Initialize WebCam
        self.video_getter = WebcamVideoStream(src=self.config.general_settings['camera_source'],
                                              width=self.config.general_settings['width'],
                                              height=self.config.general_settings['height']
                                              ).start()
        # Initialize Lidar
        self.lidar = LidarStream().start()
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # Initialize Camera Intrinsic Matrix
        K = np.array(self.config.general_settings['camera_intrinsic_matrix'])
        # Set warning threshold (default set to 2 cm)
        warning_threshold = self.config.general_settings['warning_threshold']
        # Set collision threshold (default set to 1.5 meter)
        collision_threshold = self.config.general_settings['collision_threshold']
        # Set critical warning threshold (default set to 0.5 meter)
        critical_threshold = self.config.general_settings['critical_distance']
        last_collision_distance = 50
        try:
            while True:
                if self.stopped:
                    logging.critical(f"Received shutdown signal. Closing Camera and Lidar services.")
                    # Close all service
                    self.close_service(self.video_getter, self.lidar)
                    self.stopped = False
                    return
                # Get latest video frame
                latest_frame = self.video_getter.read()
                if latest_frame is None:
                    continue
                else:
                    img_src = latest_frame
                # Detect objects in video frame
                img_src, detection = self.inferer.infer(
                    img_src,
                    self.config.ml_lib_settings['conf_thres'],
                    self.config.ml_lib_settings['iou_thres'],
                    self.config.ml_lib_settings['max_det'],
                    self.config.ml_lib_settings['hide_labels'],
                    self.config.ml_lib_settings['hide_conf']
                )
                # Read latest lidar point cloud
                scans = self.lidar.read()
                closest_objects = get_closest_objects(detection, img_src, K)
                collision_distance = measure_collision_distance(scans)
                if abs(last_collision_distance - collision_distance) >= warning_threshold \
                        and 0 < collision_distance <= collision_threshold:
                    if collision_distance <= critical_threshold:
                        self.warning_msg = {"msg": f"Please stop, possible collision ahead.", "status": "unread"}
                    else:
                        self.warning_msg = {"msg": self.create_warning_message(closest_objects, collision_distance),
                                            "status": "unread"}
                    logging.critical(f"Warning message:{self.warning_msg}")
                    last_collision_distance = collision_distance

        except KeyboardInterrupt:
            logging.critical("Keyword interrupt. Closing Camera and Lidar services.")
            self.close_service(self.video_getter, self.lidar)
        except InterruptedError:
            logging.critical("Keyword interrupt. Closing Camera and Lidar services.")
            self.close_service(self.video_getter, self.lidar)

    def stop(self):
        self.stopped = True

    @staticmethod
    def close_service(video_getter, lidar):
        video_getter.stop()
        lidar.stop()

    @staticmethod
    def create_warning_message(closest_objects, collision_distance):
        message = None
        if len(closest_objects) > 0:
            message = f"Possible {','.join(closest_objects)} ahead at distance {collision_distance} meter"
        else:
            message = f"Unidentified obstacle ahead at {collision_distance} meter distance"
        return message
