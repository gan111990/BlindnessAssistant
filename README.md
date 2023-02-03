## BlindnessAssistant
Welcome to BlindnessAssistance installation guide. Please follow all the steps in the given sequence to avoid complication during setup.

#### 1. Setup Nvidia Jetson
#### 2. Install all linux libs given below,
```bash
sudo apt update
sudo apt install python3-dev python3-pip libopenblas-dev liblapack-dev libopenblas-base libopenmpi-dev libomp-dev libfreetype6-dev libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
```
#### 3. Clone the project
```bash
git clone https://github.com/gan111990/BlindnessAssistant.git
```
#### 4. Install all required packages
```bash
cd BlindnessAssistant/
wget https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl -O torch-1.8.0-cp36-cp36m-linux_aarch64.whl
pip3 install -r requirements-jetson.txt
cd ..
git clone --branch v0.9.0 https://github.com/pytorch/vision torchvision
cd torchvision/
export BUILD_VERSION=0.9.0
python3 setup.py install --user
```
#### 5. Camera calibration
1. Print image(BlindnessAssistant/src/research/pattern_chessboard2.png) and stick it on the wall or somewhere on flat surface.
2. Connect webcam into your laptop and use any webcam app to click pictures of the pattern with different angles.
3. Replace the all img*.jpg in the directory BlindnessAssistant/src/research/ with your camera images. (Note: Capture one straight picture.)
4. You can add as many pictures as you want. Please keep the image format in `.jpg`.
5. Run below command
    ```bash
   python camera_calibration.py
    ```
Press enter when the image appears and continue for all images. 

Copy the "camera matrix" printed after camera calibration and update the "camera_intrinsic_matrix" value in the app config file.
#### 7. Update email and password in the config file with your email and password .
#### 8. Make sure all the devices are connected to the Jetson (Camera, Lidar and Wifi)
```bash
# Give Lidar special permission to run
sudo chmod 666 /dev/ttyUSB0
```
#### 9. Download the models from the Google Drive and keep it in the folder `BlindnessAssistant/data/model/`. You will find the link of the drive in the file `BlindnessAssistant/data/model/info.txt`.
#### 10. Start the app
```bash
cd BlindnessAssistant/src/web/
python3 app.py
```
#### 11. Check the shared email, you will get the app url. Now you are ready to play with the app.