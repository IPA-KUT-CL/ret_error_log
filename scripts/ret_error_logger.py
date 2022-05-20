#!/usr/bin/env python3
import re
import sys
import threading
import time
from datetime import datetime
from os import listdir
from os.path import isfile, join
from pathlib import Path

import psutil
from influxdb import InfluxDBClient


# all keywords in ros log files that need to be logged into database
detect_kwds = ['ERROR', 'FATAL']


class RET_Error_Logger():
    def __init__(self) -> None:
        print("Initialising InfluxDB client")
        self.client = InfluxDBClient(host="localhost", port="8086",
                                     username='ret', password='asdf', database="RET_Log_Error")
        self.client.create_database("RET_Log_Error")
        self.client.switch_database("RET_Log_Error")

        # path for logs
        syslog_path = '/var/log/syslog'
        roslog_path = str(Path.home()) + '/.ros/log/latest'

        # list for all files in ros log path
        roslog_list = self.list_allfiles(roslog_path)

        # open files to track
        open_syslog = open(syslog_path, 'r')
        open_rosmaster = open(join(roslog_path, 'master.log'), 'r')
        open_rosout = open(join(roslog_path, 'rosout.log'), 'r')

        # patterns to parse the log
        # for rosout.log, groups epoch time, log level, node name, location, topics
        pat_rosout = '([\d.]+)\s(\w+)\s\/(\w+)\s\[(\/*.*\.[\w:]+[\(\w+\)]+)\]\s\[[\w:]+\s[\/\w+, ]+\]\s'
        groups_rosout = ['epoch time', 'level', 'source', 'location', 'message']

        # for master.log or roslaunch*, groups source, log level, date, system time, 
        pat_master = '\[(ros\w+.\w+|xml\w+)\]\[(\w+)\]\s[\d-]+\s([\d:,]+)\s'
        groups_master = ['source', 'level', 'time', 'message']

        # for systemlog, match lines with segfault and groups system time, epoch time and program name
        pat_sys = '(\d+:\d+:\d+)[\s\w:]+\[[\d.\s]+\]\s([\w.]+)\[\d+\]:\ssegfault'
        groups_sys = ['time', 'source', 'message']

        # An event to control all threads
        self.run_thread = threading.Event()
        self.run_thread.set()

        try:
            # define all threads
            #thread_mem = threading.Thread(target=self.monitor_memory, args=(self.run_thread,), daemon=True)
            thread_sys = threading.Thread(target=self.monitor_syslog, args=(open_syslog, pat_sys, groups_sys, self.run_thread,), daemon=True)
            thread_ros_master = threading.Thread(target=self.monitor_ros, args=(open_rosmaster, pat_master, groups_master, self.run_thread,), daemon=True)
            thread_ros_rosout = threading.Thread(target=self.monitor_ros, args=(open_rosout, pat_rosout, groups_rosout, self.run_thread,), daemon=True)
            # start all threads
            #thread_mem.start()
            thread_sys.start()
            thread_ros_master.start()
            thread_ros_rosout.start()
            while True:
                pass
        except KeyboardInterrupt:
            # enable the event to stop all threads
            self.run_thread.clear()
            
            print('Joining threads')
            #thread_mem.join()
            thread_sys.join()
            thread_ros_master.join()
            thread_ros_rosout.join()

            print("close all opened files")
            open_syslog.close()
            open_rosmaster.close()
            open_rosout.close()
            sys.exit()
    
    def list_allfiles(self, path):
        '''
        list all ros log files
        '''
        return [f for f in listdir(path) if isfile(join(path, f))]

    def monitor_syslog(self, file, pattern, group_kwds, event):
        '''
        thread to monitor /var/log/syslog
        only log the line with 'segfault'
        '''
        # open_syslog = open(self.syslog_path, 'r')
        log_line = self.follow(file, event)
        for line in log_line:
            match = re.search(pattern, line)
            if match:
                line_segs = list(match.groups()) + ['segfault']
                line_dict = {group_kwds[i]: line_segs[i] for i in range(len(line_segs))}
                print(line_dict)
                self.write_log_to_influxdb(line_dict)
                    
    def monitor_memory(self, event):
        '''
        print memory use
        cf: https://stackoverflow.com/questions/276052/how-to-get-current-cpu-and-ram-usage-in-python
        '''
        print(f'monitoring memory')
        while event.is_set():
            mem_usage = psutil.virtual_memory()[2]
            print(f'memory used : {mem_usage}%')
            time.sleep(0.2)

    def monitor_ros(self, file, pattern, group_kwds, event):
        '''
        monitor all ros files and parse with given pattern
        '''
        log_line = self.follow(file, event)
        for line in log_line:
            # unless first detect if kwds in line, wait for next line
            if not any([(word in line) for word in detect_kwds]):
                continue
            # parse the line with pattern
            match = re.search(pattern, line)
            # print info if the line not matches the pattern
            if not match:
                print('log line not match pattern')
                print(line)
                continue
            else:
                # list up groups in match and rest message
                line_segs = list(match.groups()) + [line[match.end():-1]]
                # form a dict to convert to json format
                line_dict = {group_kwds[i]: line_segs[i] for i in range(len(line_segs))}
                self.write_log_to_influxdb(line_dict)

    def follow(self, file, event):
        '''
        follow an updating file
        '''
        print("following file:" + file.name)
        file.seek(0, 2)
        while event.is_set():
            line = file.readline()
            if not line:
                time.sleep(0.2)
                continue
            yield line

    def read_existing(self, file):
        '''
        to be deprecated: read all existing line
        '''
        print("reading existing lines from file:" + file.name)
        line = file.read()
        return line
    
    def write_log_to_influxdb(self, log_dict):
        '''
        write parsed log to influxdb
        '''
        source = log_dict['source']
        # get level
        if 'level' in log_dict:
            level = log_dict['level']
        else:
            # system log message not have key as level, assign as FATAL
            level = 'FATAL'
        # get time
        if 'time' in log_dict:
            log_time = log_dict['time']
        else:
            # convert epoch time to UTC
            log_time = datetime.utcfromtimestamp(float(log_dict['epoch time'])).strftime('%H:%M:%S')
        message = log_dict['message'],

        json = [{
            
            "measurement": "RET_Errors",
            "tags": {
                "source": str(source),
                "level": str(level)
            },
            
            "time": log_time,
            "fields": {
                "time": str(log_time),
                "source": str(source),
                "message": str(message)
            }
        }]
        print(json)
        self.client.write_points(json)


if __name__ == '__main__':
    logger = RET_Error_Logger()
