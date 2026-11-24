#!/usr/bin/env python3
"""Rewrap the dashboard's Record button videos as they arrive in ~/Downloads, so players show their full length.

Started with the dashboard by the planner launch files. Chrome saves the videos as fragmented MP4 with no index of the
whole video; once a new *_voxels.mp4 or *_paths.mp4 has finished downloading, this copies it into a normal MP4 without
re-encoding (fix_recordings.sh does the same by hand).
"""
import glob
import os
import subprocess
import time

import rospy

WATCH = os.path.expanduser('~/Downloads')
PATTERNS = ('*_voxels*.mp4', '*_paths*.mp4') # Chrome adds ' (1)' to a name it already saved


def fragmented(path):
    with open(path, 'rb') as f:
        return b'moof' in f.read(1 << 20) # the first fragments sit near the start


def fix(path):
    tmp = path[:-4] + '.fixing.mp4'
    done = subprocess.run(['ffmpeg', '-loglevel', 'error', '-y', '-i', path, '-c', 'copy', '-movflags', '+faststart', tmp])
    if done.returncode == 0:
        os.replace(tmp, path)
        rospy.loginfo('[recording_fixer] Fixed %s', path)
    else:
        if os.path.exists(tmp): os.remove(tmp)
        rospy.logwarn('[recording_fixer] Could not fix %s', path)


def main():
    rospy.init_node('recording_fixer')
    started = time.time()
    last_size, done = {}, {} # path -> size at the last look; path -> modification time when it was dealt with
    while not rospy.is_shutdown():
        for pattern in PATTERNS:
            for path in glob.glob(os.path.join(WATCH, pattern)):
                if path.endswith('.fixing.mp4'): continue
                try:
                    stat = os.stat(path)
                except OSError:
                    continue
                if stat.st_mtime < started - 5 or done.get(path) == stat.st_mtime: continue # older, or already dealt with
                if last_size.get(path) == stat.st_size: # finished downloading: unchanged for 2 s
                    if fragmented(path): fix(path)
                    done[path] = os.stat(path).st_mtime
                last_size[path] = stat.st_size
        rospy.sleep(2.0)


if __name__ == '__main__':
    main()
