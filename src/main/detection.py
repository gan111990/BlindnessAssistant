#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import argparse
import os
import sys
import os.path as osp

import time
import numpy as np
from collections import deque
import torch
import cv2

SRC_PATH = osp.join('..', '..', 'src')
sys.path.insert(0, SRC_PATH)

from main.configuration import Configuration
from yolov6.utils.events import LOGGER
from yolov6.core.inferer_video import Inferer
from lidar.LidarStream import LidarStream
from video.VideoShow import VideoShow
from video.WebcamVideoStream import WebcamVideoStream


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

    cv2.useOptimized()
    is_interrupted = False
    headless_flag = config.general_settings['headless']
    video_getter = WebcamVideoStream(src=config.general_settings['camera_source'],
                                     width=config.general_settings['image_size'],
                                     height=config.general_settings['image_size']).start()

    # enable video show thread only if headless functionality is disabled
    if headless_flag == False:
        video_shower = VideoShow(video_getter.read()).start()

    # Inference
    inferer = Inferer(
        config.ml_lib_settings['weights'],
        config.ml_lib_settings['device'],
        config.ml_lib_settings['yaml'],
        config.general_settings['image_size']
    )
    fps_calculator = CalcFPS()
    try:
        while True:
            latest_frame = video_getter.read()
            if latest_frame is None:
                continue
            else:
                img_src = latest_frame
            t1 = time.time()
            img_src = inferer.infer(
                img_src,
                config.ml_lib_settings['conf_thres'],
                config.ml_lib_settings['iou_thres'],
                config.ml_lib_settings['max_det'],
                config.ml_lib_settings['hide_labels'],
                config.ml_lib_settings['hide_conf']
            )
            t2 = time.time()
            # FPS counter
            fps_calculator.update(1.0 / (t2 - t1))
            avg_fps = fps_calculator.accumulate()
            inferer.draw_text(
                img_src,
                f"FPS: {avg_fps:0.1f}",
                pos=(20, 20),
                font_scale=1.0,
                text_color=(204, 85, 17),
                text_color_bg=(255, 255, 255),
                font_thickness=2,
            )
            if headless_flag == False:
                video_shower.frame = img_src
            LOGGER.critical(f"FPS: {avg_fps:0.1f}")
    except Exception as e:
        LOGGER.critical(f'Exception occurred in video DMS:{e}')
        cv2.destroyAllWindows()
        video_getter.stop()
        if headless_flag == False:
            video_shower.stop()
        sys.exit()

    # Normal stopping
    cv2.destroyAllWindows()
    video_getter.stop()
    if headless_flag == False:
        video_shower.stop()

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
