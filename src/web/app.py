from flask import Flask, render_template, request, send_file
import os
import sys
import cv2
import logging
from gtts import gTTS

find_src = lambda p: os.path.sep.join(p[:p.index('src') + 1])
SRC_PATH = find_src(os.path.realpath(__file__).split(os.path.sep))
if SRC_PATH in sys.path:
    pass
else:
    sys.path.insert(0, SRC_PATH)

from main.configuration import Configuration
from main.main import Main
from yolov6.core.inferer_video import Inferer
from util.Email import Email
from util.util import create_app_url

app = Flask(__name__, static_folder='../web', template_folder='.')
config = Configuration('./../../data/config/config.json')
cv2.useOptimized()
# Inference
inferer = Inferer(
    config.ml_lib_settings['weights'],
    config.ml_lib_settings['device'],
    config.ml_lib_settings['yaml'],
    config.general_settings['width']
)
msg = f"Hi, BlindnessAssistant app latest url is {create_app_url()}"
main = Main(inferer, config)
email = Email(config.general_settings['sender_email'], config.general_settings['sender_email_pwd'])
email.send_email(config.general_settings['user_email'], "BlindnessAssistant info", msg)


@app.route('/')
def index():
    """Blindness Assistant home page."""
    return render_template('index.html')


@app.route('/control', methods=['POST'])
def app_control():
    if request.form.get('action') == 'on':
        logging.critical(f"Request to start app.")
        main.start()
        main.warning_msg = {"msg": "App started", "status": "unread"}
        logging.critical(f"App started successfully.")
    else:
        logging.critical(f"Request to stop app.")
        main.stop()
        main.warning_msg = {"msg": "App stopped", "status": "unread"}
        logging.critical(f"App stopped successfully.")
    return 'Success'


@app.route('/warning')
def get_warning():
    """Get updated warning"""
    if main.warning_msg['status'] == 'unread':
        main.warning_msg['status'] = 'read'
        msg = main.warning_msg['msg']
        obj = gTTS(text=msg, lang='en', slow=False)
        obj.save("./audio/warning.mp3")
        url = './audio/warning.mp3'
    else:
        url = './audio/empty.mp3'
    return send_file(url, mimetype='audio/mpeg')


if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8000)
