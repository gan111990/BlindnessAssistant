import os
import sys

import time
import numpy as np
from collections import deque
import torch
import cv2
import logging

find_src = lambda p: os.path.sep.join(p[:p.index('src') + 1])
SRC_PATH = find_src(os.path.realpath(__file__).split(os.path.sep))
if SRC_PATH in sys.path:
    pass
else:
    sys.path.insert(0, SRC_PATH)

from main.configuration import Configuration
from yolov6.utils.events import LOGGER
from yolov6.core.inferer_video import Inferer
# from lidar.LidarStream import LidarStream
from lidar.LidarStreamPylidar import LidarStream
from video.VideoShow import VideoShow
from video.WebcamVideoStream import WebcamVideoStream
from util.util import project_lidar_to_camera, get_closest_objects

K = None

def interrupt_signal_handler(self, signum, stack):
    self.is_interrupted = True

@torch.no_grad()
def run(config_file_name):
    config = Configuration(config_file_name)
    # create save dir
    # if save_dir is None:
    #     save_dir = osp.join(project, name)
    #     save_txt_path = osp.join(save_dir, 'labels')
    # else:
    #     save_txt_path = save_dir
    # if (save_img or save_txt) and not osp.exists(save_dir):
    #     os.makedirs(save_dir)
    # else:
    #     LOGGER.warning('Save directory already existed')
    K = np.array(config.general_settings['camera_intrinsic_matrix'])
    cv2.useOptimized()
    # Inference
    inferer = Inferer(
        config.ml_lib_settings['weights'],
        config.ml_lib_settings['device'],
        config.ml_lib_settings['yaml'],
        config.general_settings['width']
    )
    is_interrupted = False
    headless_flag = config.general_settings['headless']
    video_getter = WebcamVideoStream(src=config.general_settings['camera_source'],
                                     # width=config.general_settings['image_size'],
                                     # height=config.general_settings['image_size']
                                     ).start()

    # enable video show thread only if headless functionality is disabled
    if headless_flag == False:
        video_shower = VideoShow(video_getter.read()).start()
    # Initialize Lidar
    lidar = LidarStream()
    lidar.start()
    fps_calculator = CalcFPS()
    try:
        while True:
            latest_frame = video_getter.read()
            if latest_frame is None:
                continue
            else:
                img_src = latest_frame
            t1 = time.time()
            img_src, detection = inferer.infer(
                img_src,
                config.ml_lib_settings['conf_thres'],
                config.ml_lib_settings['iou_thres'],
                config.ml_lib_settings['max_det'],
                config.ml_lib_settings['hide_labels'],
                config.ml_lib_settings['hide_conf']
            )
            scans = lidar.read()
            # print(pprint.pformat(scans))
            # detection = object_to_lidar_angle_mapping(detection, scans)
            detection = project_lidar_to_camera(img_src, detection, scans, K)
            print(detection)
            get_closest_objects(detection, img_src, K)
            t2 = time.time()
            # FPS counter
            fps_calculator.update(1.0 / (t2 - t1))
            avg_fps = fps_calculator.accumulate()
            if headless_flag == False:
                video_shower.frame = img_src
            LOGGER.critical(f"FPS: {avg_fps:0.1f}")
    except KeyboardInterrupt:
        close_service(video_shower, video_getter, lidar, headless_flag)
        sys.exit()
    except InterruptedError:
        close_service(video_shower, video_getter, lidar, headless_flag)
        sys.exit()

    # Normal stopping
    close_service(video_shower, video_getter, lidar, headless_flag)
    sys.exit()


def close_service(video_shower, video_getter, lidar, headless_flag):
    cv2.destroyAllWindows()
    if headless_flag == False:
        video_shower.stop()
    video_getter.stop()
    lidar.stop()


class CalcFPS:
    def __init__(self, nsamples: int = 50):
        self.framerate = deque(maxlen=nsamples)

    def update(self, duration: float):
        self.framerate.append(duration)

    def accumulate(self):
        if len(self.framerate) > 1:
            return np.average(self.framerate)
        else:
            return 0.0


if __name__ == "__main__":
    run('./../../data/config/config.json')
