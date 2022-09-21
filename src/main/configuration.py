import json
import logging
import shutil
import os
import sys
import copy

SRC_PATH = os.path.join('..', '..', 'src')
sys.path.insert(0, SRC_PATH)


class Configuration:
    """
    Configuration reader
    """
    def __init__(self, config_file_name=None, all_settings=None):
        """
        Constructor
        :param config_file_name: input json file containing all settings
        """
        # run locally, parse settings from local file
        if all_settings is None:
            self.config_file_name = config_file_name
            self.all_settings = self.parse_json_config()
        else: # run on cloud, parse settings from rest call
            self.all_settings = all_settings

        self.general_settings = self.all_settings['general_settings']
        self.ml_lib_settings = self.all_settings['ml_lib_settings']
        self.mqtt_settings = self.all_settings['mqtt_setting']
        self.pipeline_settings = self.all_settings['pipeline']
        self.log_to_console = self.pipeline_settings['logging']['log_to_file']

    def parse_json_config(self):
        """
        Parse json file
        :return: json settings
        """
        with open(self.config_file_name) as json_data:
            data = json.load(json_data)
        return copy.deepcopy(data)

    def initalise_directories(self, path_to_dir, directory_name=None, remove_dir=False):
        """
        Initialise directories for local storage
        :param path_to_dir: directory path
        :param directory_name: directory name
        :param remove_dir: remove directory
        :return: void
        """
        if directory_name is None:
            new_dir = path_to_dir
        else:
            new_dir = path_to_dir + directory_name

        # delete dir
        if os.path.exists(new_dir) and remove_dir:
            shutil.rmtree(new_dir)

        # create dirs
        if not os.path.exists(new_dir):
            os.makedirs(new_dir)

    def get_log_level(self, level):
        """
        Get log level
        :param level: logging level str
        :return: return logging object level
        """
        if level == "error":
            return logging.ERROR
        elif level == "debug":
            return logging.DEBUG
        elif level == "info":
            return logging.INFO
        elif level == "warning":
            return logging.WARNING
        elif level == "critical":
            return logging.CRITICAL

    def setup_logging(self, level, log_to_file=False, log_file='dms.log'):
        """
        Setup logging
        :param level: (critical, debug, info etc)
        :param log_to_file: output logs to a file or in console
        :param log_file: file name for storing logs
        :return: void
        """
        logger = logging.getLogger()
        logger.setLevel(self.get_log_level(level))
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')

        if self.log_to_console == True:
            console_log_handler = logging.StreamHandler(sys.stdout)
            console_log_handler.setFormatter(formatter)
            logger.addHandler(console_log_handler)

        if log_to_file == True:
            file_handler = logging.FileHandler(log_file)
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)
