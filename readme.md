# RET Logger
This project is to record and examine all logs generated while running ros applications, including syslog and logs by ROS. The core is a stand-alone python script [ret_error_logger.py](./scripts/ret_error_logger.py) to track and examine log files and write to database.

## Features
1. Multithread to track several log files concurrently
2. Using Regex to parse log line
3. Reformatting log lines in to json and write into influxdb
4. Easy examples of segmentation fault in both cpp and python with or without ROS to test

## How to use
- Start influx service at the beginning
    ```
    sudo service influx start
    ```
- Put the keywords to be recorded in [ret_error_logger.py](./scripts/ret_error_logger.py#L18)
- Launch a ROS program, for example
    ```
    roslaunch ret_error_logger segfault_launch_py.launch
    ```
- Run the error logger by
    ```
    cd /path/to/project/scripts
    python3 ret_error_logger.py
    ```
## TODO:
- [ ] ROS logs recorded in UTC time, change to local time
- [ ] Only tracking master.log and rosout.log for now, generalize to all ROS log files
- [ ] Only tracking segfault in syslog for now, track other types of fault?
- [ ] Automatically switch to latest log directory, so that this script can keep running while changing ROS programs


## Sources
1. [Programiz: Tutorial on Regex](https://www.programiz.com/python-programming/regex)
2. [Regular Expression 101](https://regex101.com/)
3. [Influxdb Python API](https://influxdb-python.readthedocs.io/en/latest/index.html)