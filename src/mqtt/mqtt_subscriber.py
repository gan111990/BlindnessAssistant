import sys
import os.path

find_src = lambda p: os.path.sep.join(p[:p.index('src') + 1])
SRC_PATH = find_src(os.path.realpath(__file__).split(os.path.sep))
if SRC_PATH in sys.path:
    pass
else:
    sys.path.insert(0, SRC_PATH)

import argparse
import paho.mqtt.client as mqtt
import pygame
import time
import logging as log


class MQTTSubscriber:

    def __init__(self, client):
        self.client = client
        self.stop = False

    def enable_audible_alert(self, sound_file_path='./../../../media/warning_small.wav'):
        """
        Play a sound using the file

        Args:
            sound_file_path: input file path for the sound alert
        Returns:
            void
        """
        pygame.mixer.init()
        pygame.mixer.music.load(sound_file_path)
        pygame.mixer.music.play()

    def mqtt_subscribe(self, topic, sound_file_path='./../../../media/warning_small.wav'):
        """
        Subscribe mqtt topic and read related message
        Args:
            topic: mqtt topic
        Returns:
            void
        """

        def on_message(client, userdata, message):
            log.critical(f'message received {str(message.payload.decode("utf-8"))}')
            log.critical(f'message topic={message.topic}')
            if message.topic == "dms_alarm1":
                # enable_audible_alert()
                pygame.mixer.init()
                pygame.mixer.music.load(sound_file_path)
                pygame.mixer.music.play()

        self.client.on_message = on_message
        self.client.loop_start()
        log.info(f'Going to subscribe topic={topic}')
        self.client.subscribe(topic)
        # time.sleep(10)
        # while not self.stop:
        # continue

    def stop_subscriber(self):
        self.stop = True
        self.client.loop_stop()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-ba", "--broker_address", required=False, help="MQTT broker address")
    # broker_address = 'localhost'#'10.42.0.1'
    args = vars(ap.parse_args())
    broker_address = args["broker_address"] if args["broker_address"] \
                                               is not None else "localhost"
    client = mqtt.Client("P4")
    client.connect(broker_address)
    sub = MQTTSubscriber(client)
    sub.mqtt_subscribe('alarm1')
    time.sleep(200)
    # log.critical('Going to stop subscriber.')
    sub.stop_subscriber()
