#!/usr/bin/env python3
"""Cut every run video of a run_batch.py batch down to the exploration phase, speed it up and save it as a short GIF.

  make_gifs.py ~/mission_runs/batch_x                 # every run, about 10 s each
  make_gifs.py ~/mission_runs/batch_x --seconds 8 --width 800

The exploration phase runs from the mission CSV's first EXPLORING state to its first RETURNING (or LANDING, or the end),
placed in the video with the same clocks run_highlights.py uses. The originals are left alone; each run gets
<batch>/gifs/<run>.gif, <run>_white.gif (colors inverted, for white slides) and the clip as <run>.mp4, sharper and smaller.
"""
import argparse
import csv
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_highlights import Timeline, sr_files # noqa: E402


def exploration_span(csv_path):
    """(start, end) of the exploration phase on the mission clock, or None."""
    start = end = last = None
    for row in csv.DictReader(open(csv_path)):
        if row.get('type') != 'height': continue
        t, state = float(row['t_s']), row['state']
        last = t
        if start is None and state == 'EXPLORING': start = t
        if start is not None and end is None and state in ('RETURNING', 'LANDING'): end = t
    if start is None: return None
    return start, (end if end is not None else last)


def white_gif(clip):
    """<run>_white.gif: the clip with its colors inverted, for white slides."""
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', clip, '-vf',
                    'negate,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4',
                    '-loop', '0', clip[:-4] + '_white.gif'], check=True)


def make(run_dir, out_dir, seconds, width, fps):
    run = os.path.basename(run_dir)
    csvs = sr_files(run_dir)
    if not csvs: return 'no mission CSV'
    span = exploration_span(csvs[0])
    if not span: return 'never explored'
    timeline = Timeline(run_dir)
    video, t0 = timeline.video_of(timeline.wall_of(span[0]))
    _, t1 = timeline.video_of(timeline.wall_of(span[1]))
    if not video: return 'no video'
    t0 = max(t0, 0.0)
    length = t1 - t0
    if length <= 0: return 'empty span'
    speed = max(length / seconds, 1.0)
    src = os.path.join(run_dir, video)
    # the length goes on the output: on the input, ffmpeg 4.2's fps filter pads the sped-up clip back to full length
    cut = ['ffmpeg', '-y', '-loglevel', 'error', '-ss', '%.2f' % t0, '-i', src, '-t', '%.2f' % (length / speed)]
    scale = 'setpts=PTS/%.4f,fps=%d,scale=%d:-2:flags=lanczos' % (speed, fps, width)
    gif = os.path.join(out_dir, run + '.gif')
    subprocess.run(cut + ['-vf', scale + ',split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4',
                          '-loop', '0', gif], check=True)
    subprocess.run(cut + ['-vf', scale, '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20',
                          '-movflags', '+faststart', os.path.join(out_dir, run + '.mp4')], check=True)
    white_gif(os.path.join(out_dir, run + '.mp4'))
    return 'exploring %.0f s of video (mission %.0f-%.0f s), %.1fx -> %s (%.1f MB)' % (
        length, span[0], span[1], speed, os.path.basename(gif), os.path.getsize(gif) / 1e6)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('batch', help='a run_batch.py output folder')
    parser.add_argument('--seconds', type=float, default=10.0, help='length of each GIF')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--fps', type=int, default=12)
    parser.add_argument('--out', help='output folder (default <batch>/gifs)')
    args = parser.parse_args()
    out_dir = args.out or os.path.join(args.batch, 'gifs')
    os.makedirs(out_dir, exist_ok=True)
    for name in sorted(os.listdir(args.batch)):
        run_dir = os.path.join(args.batch, name)
        if not os.path.exists(os.path.join(run_dir, 'run_info.json')): continue
        try:
            print('%-30s %s' % (name, make(run_dir, out_dir, args.seconds, args.width, args.fps)), flush=True)
        except subprocess.CalledProcessError as e:
            print('%-30s ffmpeg failed: %s' % (name, e), flush=True)


if __name__ == '__main__':
    main()
