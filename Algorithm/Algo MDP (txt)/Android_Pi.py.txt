import configparser
from time import sleep
import serial
import os

config = configparser.ConfigParser()
config.read('configs/PI_config.ini')

class Android_Pi:

    bd_addr = config['android_connection']['bd_addr']
    port = config['android_connection']['port']

    def __init__(self) -> None:
        pass

    def connect(self):
        self.ser = serial.Serial(self.bd_addr)

    def send(self, message):
        os.system(f'echo {message} >> {self.bd_addr}')

    def recv(self, persistent=False):

        retry = 10
        data = None

        while True:
            temp_data = os.popen(f"cat {self.bd_addr}").read()
            if temp_data == '':
                sleep(0.25)
                retry -= 1

                if not persistent and retry < 0:
                    return None

                continue

            data = temp_data
            break

        return data
