import multiprocessing
import time
import algorithm
import Android_Pi
import task1
import task2

android_pi = Android_Pi.Android_Pi()
android_pi.connect()
task1 = task1.TaskOne()
task2 = task2.TaskTwo()
android_command = None
algo_path = None

print("Awaiting android command")
try:
    while android_command is None or len(android_command) == 0:
        android_command = android_pi.recv(True)
        if android_command == "BEGIN": # start task 1
            task1.connect_to_STM()
            process = multiprocessing.Process(target=task1.execute_commands())
            process.start()
            print("Joining task 1")
            process.join(345)
            if process.is_alive():
                task1.end_imagerec()
                print("From Main: Function took too long! Terminating...")
                process.terminate()
                process.join()
            else:
                task1.end_imagerec()
                print("From Main: Task 1 completed")


        elif android_command == "BEGIN2": # start task 2
            task2.connect_to_STM()
            process = multiprocessing.Process(target=task2.start_taskTwo()) #update with relevant function
            process.start()
            process.join(165)
            if process.is_alive():
                task2.end_imagerec()
                print("Function took too long! Terminating...")
                process.terminate()
                process.join()
            else:
                task2.end_imagerec()
                print("Task 2 completed")

        elif android_command: # android sent obstacles data
            # algo compute path
            obstacles = android_command.split("|")
            print("Recv obstacles")
            algo = algorithm.Algorithm(obstacles, False)
            print("Computing path")
            algo_path = algo.computePath()
            print("Path computed")
            print(algo_path)
            task1.set_command_by_algo(algo_path)
            android_command = None

except e:
    print(e)


