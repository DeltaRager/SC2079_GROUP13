import time
import os
import serial

class TaskOne:
    def __init__(self):
        self.command_by_algo = [] # ideally all lists should look the same
        self.command_sent = [] # eg.FW30, 'FR10', 'C,3'
        self.command_done = [] # eg.FW30, FR10...
        self.obstacle_number_list = []
        self.index_command_by_algo = 0
        self.index_command_sent = 0
        self.index_command_done = 0
        self.index_obstacle_number_list = 0

        self.commands_with_STM = {'F':'FW10', 'B':'BW10', 'FL':'FL30', 'FR':'FR30', 'BL':'BL30', 'BR':'BR30', 'STOP':'STOP'}
        self.commands_with_Android = {'FW10': 'MOVE,FORWARD', 'BW10': 'MOVE,BACKWARD', 'FL30': 'TURN,FORWARD_LEFT', 
            'FR30': 'TURN,FORWARD_RIGHT', 'BL30': 'TURN,BACKWARD_LEFT', 'BR30': 'TURN,BACKWARD_RIGHT'}

        #self.ack_char = b'ACK'
        self.acknowledged = False

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

    def set_command_by_algo(self, algo_path):
        self.command_by_algo = algo_path

    def write_to_STM(self, command):
        if 'L' in command or 'R' in command: #FL, FR, BL, BR
            self.ser.write('RS00'.encode('utf-8'))
            print("Sent to STM is RS00")
            time.sleep(1)
            command_to_be_sent = command
            self.ser.write(command_to_be_sent.encode('utf-8'))
            print(f"Command is {command}, Sent to STM is {command_to_be_sent}")
            self.command_sent.append(command_to_be_sent)
            print(f"Updated command_sent: {self.command_sent}")
        elif 'C' not in command: #FW, BW
            command_to_be_sent = command
            self.ser.write(command_to_be_sent.encode('utf-8'))
            print(f"Command is {command}, Sent to STM is {command_to_be_sent}")
            self.command_sent.append(command_to_be_sent)
            print(f"Updated command_sent: {self.command_sent}")
        else:
           # print("-----------------------------------Debug here----------------------------------")
           # print(f"command is {command}")
            obstacle_number = int(command[-1])
           # print(f"obstacle number is {obstacle_number}")
            self.obstacle_number_list.append(obstacle_number)
           # command = 'STOP'
           # command_to_be_sent = self.commands_with_STM[command]
           # self.ser.write(command_to_be_sent.encode('utf-8'))
           # print("Append is successful")
            print(f"Added Checkpoint")
            self.command_sent.append(command)
            print(f"Updated command_sent: {self.command_sent}")

    def execute_commands(self):
        #writing first command to stm
        print("writing first command")
        self.write_to_STM(self.command_by_algo[self.index_command_by_algo])
        self.index_command_by_algo += 1

        while True:
            if self.index_command_done == len(self.command_by_algo):
                print("All commands carried out, ending...")
                break
            if 'C' in self.command_sent[-1]:
                self.process_acknowledgement()
            else:
                s = self.ser.readline().decode('utf-8')
                print(f"Received from STM: {s}")
                if s == b'':
                    print("Timeout, executing next command.")
                    s = 'ACK'
                if 'ACK' in s:
                    self.acknowledged = True
                if self.acknowledged:
                    self.process_acknowledgement()

    def process_acknowledgement(self):
        command_that_received_ack = self.command_sent[self.index_command_sent]
        self.command_done.append(command_that_received_ack)
        print(f"Updated command_done: {self.command_done}")

        if self.index_command_sent <= len(self.command_sent) - 1:
            print(f"Command to be carried out: {self.command_sent[self.index_command_sent]}")
            self.index_command_sent += 1
        self.index_command_done += 1

        if 'C' not in command_that_received_ack:
            if 'FW' in command_that_received_ack:
                steps_taken = int(command_that_received_ack[2])
                robot_movement_android = f"echo {self.commands_with_Android['FW10']},{steps_taken} > /dev/rfcomm0"
            elif 'BW' in command_that_received_ack:
                steps_taken = int(command_that_received_ack[2])
                robot_movement_android = f"echo {self.commands_with_Android['BW10']},{steps_taken} > /dev/rfcomm0"
            else:
                steps_taken = 1
                robot_movement_android = 'echo ' + self.commands_with_Android[command_that_received_ack] + ' > /dev/rfcomm0'
            print(f"{robot_movement_android} will be called by system to send to Android {steps_taken} times")
            os.system(robot_movement_android)
            print(f"{robot_movement_android} called")
            if self.index_command_by_algo <= len(self.command_by_algo) - 1:
                self.write_to_STM(self.command_by_algo[self.index_command_by_algo])
                self.index_command_by_algo += 1

        else:
            self.handle_stop_command()


        self.acknowledged = False

    def handle_stop_command(self):
        print("Command that received ack is 'STOP'")
        comms_file = '/home/pi/shared/comms1.txt'
        last_size = os.path.getsize(comms_file) if os.path.exists(comms_file) else 0
        print("Going to take picture now...")
        os.system('libcamera-still -o /home/pi/shared --datetime -t 500')

        while True:
            current_size = os.path.getsize(comms_file)
            if current_size > last_size:
                print("Ack received, current_size > last_size")
                with open(comms_file, 'r+') as f:
                    image_id = f.readlines()
                    f.truncate(0)

                image_id = str(image_id)[2:4]  # extract image ID
                print(f"obstacle number list: {self.obstacle_number_list}")
                print(f"index for ^ is {self.index_obstacle_number_list}")
                content_to_be_called = f'echo TARGET,{self.obstacle_number_list[self.index_obstacle_number_list]},{image_id} > /dev/rfcomm0'
                self.index_obstacle_number_list += 1
                print(f"{content_to_be_called} will be called by system to send to Android")
                os.system(content_to_be_called)
                print(f"{content_to_be_called} called")

                if self.index_command_by_algo <= len(self.command_by_algo) - 1:
                    self.write_to_STM(self.command_by_algo[self.index_command_by_algo])
                    self.index_command_by_algo += 1
                break

    def end_imagerec(self):
        print("Adding END to file")
        os.system('touch /home/pi/shared/END')
        time.sleep(7)
        print("Deleting END from file")
        os.system('rm /home/pi/shared/END')

