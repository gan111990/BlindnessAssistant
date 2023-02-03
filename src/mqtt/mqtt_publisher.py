import sys
import os.path

find_src = lambda p: os.path.sep.join(p[:p.index('src') + 1])
SRC_PATH = find_src(os.path.realpath(__file__).split(os.path.sep))
if SRC_PATH in sys.path:
    pass
else:
    sys.path.insert(0, SRC_PATH)

import paho.mqtt.client as mqtt
import logging as log


class MQTTPublisher:

    def __init__(self, client):
        self.client = client

    def mqtt_publish(self, topic, message):
        log.info(f'Going to publish topic={topic} message={message}')
        self.client.publish(topic, message)


if __name__ == '__main__':
    broker_address = 'localhost'  # '10.42.0.1'
    client = mqtt.Client("P1")
    client.connect(broker_address)
    pub = MQTTPublisher(client)
    pub.mqtt_publish('dms_alarm1', 'test')