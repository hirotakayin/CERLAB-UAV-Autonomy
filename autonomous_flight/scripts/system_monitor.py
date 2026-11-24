#!/usr/bin/env python3

# FILE: system_monitor.py
# ------------------------
# publishes this computer's memory use for the mission dashboard: a browser page cannot read it
import glob
import json
import os
from collections import Counter

import rospy
from std_msgs.msg import String

PAGE_MB = os.sysconf('SC_PAGE_SIZE') / 1048576.0


def read_memory():
    info = {}
    with open('/proc/meminfo') as f:
        for line in f:
            key, value = line.split(':')
            info[key] = int(value.split()[0]) / 1024.0 # kB to MB
    return {'total_mb': info['MemTotal'], 'used_mb': info['MemTotal'] - info['MemAvailable'],
            'swap_total_mb': info['SwapTotal'], 'swap_used_mb': info['SwapTotal'] - info['SwapFree']}


def read_top(count=4):
    # by program name, so a browser's many processes count as one. Pages they share count more than once
    sizes = Counter()
    for path in glob.glob('/proc/[0-9]*'):
        try:
            with open(path + '/comm') as f:
                name = f.read().strip()
            with open(path + '/statm') as f:
                sizes[name] += int(f.read().split()[1]) * PAGE_MB
        except (OSError, IndexError, ValueError):
            pass # the process ended
    return [{'name': name, 'mb': mb} for name, mb in sizes.most_common(count)]


if __name__ == '__main__':
    rospy.init_node('system_monitor')
    pub = rospy.Publisher('/exploration/system', String, queue_size=1, latch=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        status = read_memory()
        status['top'] = read_top()
        pub.publish(String(json.dumps(status)))
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
