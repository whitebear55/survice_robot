#!/usr/bin/env python3
import subprocess

if __name__ == '__main__':
    command = "roslaunch mero_bringup gui.launch"
    process = subprocess.Popen(command, shell=True)
    process.wait()
        
