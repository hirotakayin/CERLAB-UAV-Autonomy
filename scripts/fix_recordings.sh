#!/bin/bash
# Rewrap the dashboard's Record button videos so players show their full length and can seek.
# Chrome writes them as fragmented MP4 with no index of the whole video; this copies the video into a normal MP4,
# without re-encoding, and replaces the original. Files already fixed are skipped.
#
#   rosrun autonomous_flight fix_recordings.sh                  # every *_voxels.mp4 and *_paths.mp4 in ~/Downloads
#   rosrun autonomous_flight fix_recordings.sh a.mp4 b.mp4      # just these
if [ $# -eq 0 ]; then
	shopt -s nullglob
	set -- ~/Downloads/*_voxels.mp4 ~/Downloads/*_paths.mp4
fi
for f in "$@"; do
	if ! grep -q -a -m1 moof "$f"; then
		echo "Already fine: $f"
		continue
	fi
	tmp="${f%.mp4}.fixing.mp4"
	if ffmpeg -loglevel error -y -i "$f" -c copy -movflags +faststart "$tmp"; then
		mv "$tmp" "$f"
		echo "Fixed: $f"
	else
		rm -f "$tmp"
		echo "Could not fix: $f"
	fi
done
