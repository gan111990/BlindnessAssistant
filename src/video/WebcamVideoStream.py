from threading import Thread
import cv2
import sys


class WebcamVideoStream:
    def __init__(self, src=0, width=640, height=480):
        # initialize the video camera stream and read the first frame
        # from the stream
        if type(src) == int:
            self.stream = cv2.VideoCapture(src)
            self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        else:
            self.stream = cv2.VideoCapture(src)

        (self.grabbed, self.frame) = self.stream.read()
        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                if self.stream is not None:
                    # cv2.destroyAllWindows()
                    self.stream.release()
                    sys.exit()
                return
            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True


if __name__ == "__main__":
    from VideoShow import VideoShow
    videoStream = WebcamVideoStream()
    videoStream.start()
    videoShow = VideoShow(videoStream.read()).start()
    try:
        while True:
            frame = videoStream.read()
            if frame is not None:
                videoShow.frame = frame
    except KeyboardInterrupt:
        videoShow.stop()
        videoStream.stop()
