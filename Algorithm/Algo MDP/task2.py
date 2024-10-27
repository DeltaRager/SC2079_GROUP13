import time
import os
import serial

class TaskTwo:
    def __init__(self):
        self.obstacle_num = 1
        self.left_one = ['A045', 'FW05', 'C045', 'C045', 'FW05', 'A045', 'DT20']
        self.right_one = ['C045', 'FW05', 'A045', 'A045', 'FW05', 'C045', 'DT20']
       # self.left_two = []
       # self.right_two = ['FR30', 'IR50', 'FL30', 'FW05', 'A045', 'IR00', 'A045', 'FW05', 'A045', 'DT20', 'A045', 'FR30']
        pass

    def connect_to_STM(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200)
            time.sleep(2)
            print("Connected to STM")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            exit(1)

        time.sleep(2)  # wait a while for connection to stabilize
        print("STM Ready to receive")

    def write_to_STM(self, command):
        self.ser.write(command.encode('utf-8'))
        print(f"Sent to STM: {command.encode('utf-8')}")

    def start_taskTwo(self):
        self.write_to_STM('DT20') # starting point to 1st obstacle
        while True:
            obstacle = self.ser.readline().decode('utf-8')
            if self.obstacle_num >= 3:
                print("RPI Ending...")
                time.sleep(30)
                return None
            if 'ACK' in obstacle:
               self.handle_pic_command()

    def handle_pic_command(self):
        comms_file = '/home/pi/shared/comms1.txt'
        last_size = os.path.getsize(comms_file) if os.path.exists(comms_file) else 0
        print("Going to take picture now...")
        os.system('libcamera-still -o /home/pi/shared --datetime -t 250')
        while True:
            index = 0
            current_size = os.path.getsize(comms_file)
            if current_size > last_size:
                print("Ack received, current_size > last_size")
                with open(comms_file, 'r+') as f:
                    image_id = f.readlines()
                    f.truncate(0)
                image_id = str(image_id)[2:4]  # extract image ID
                if image_id == '38':  #msg to turn right
                    if self.obstacle_num == 1:
                        print(f"Obstacle number {self.obstacle_num} is RIGHT arrow")
                        for i in range(len(self.right_one)):
                            self.write_to_STM(self.right_one[index])
                            index += 1
                            time.sleep(2)
                        self.obstacle_num += 1
                    elif self.obstacle_num == 2:
                        print(f"Obstacle number {self.obstacle_num} is RIGHT arrow")
                        self.write_to_STM('LR00')
                        self.obstacle_num += 1
                    else:
                        print("--------------------WUT, go debug (this is for turning right)--------------------")
                elif image_id == '39': #msg to turn left
                    if self.obstacle_num == 1:
                        print(f"Obstacle number {self.obstacle_num} is LEFT arrow")
                        for i in range(len(self.left_one)):
                            self.write_to_STM(self.left_one[index])
                            index += 1
                            time.sleep(2)
                        self.obstacle_num += 1
                    elif self.obstacle_num == 2:
                        print(f"Obstacle number {self.obstacle_num} is LEFT arrow")
                        self.write_to_STM('LL00')
                        self.obstacle_num += 1
                    else:
                        print("--------------------WUT, go debug (this is for turning left)--------------------")
                else:
                    print('Something was wrong with getting image id')
                break

    def end_imagerec(self):
        print("Adding END to file")
        os.system('touch /home/pi/shared/END')
        time.sleep(7)
        print("Deleting END from file")
        os.system('rm /home/pi/shared/END')
