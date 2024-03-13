from mayfly import sensor

class VideoCapture:
    def __init__(self, streamIds, configFilename=''):
        self.app = sensor.AppSensor()
        self.queue = self.app.create_event_queue(streamIds, 1)
        self.app.start(configFilename)
    def __del__(self):
        self.app.stop()
    def read(self):
        while True:
            res = self.queue.wait(1000000)
            if res is None:
                print('VideoCapture wait')
            else:
                return res;
