#!/bin/bash
# Start CERLAB UAV Mission Control and open it in its own window. Ctrl-C here stops the server and every simulation task cleanly
# (drone tasks keep running on the drone).
cd "$(dirname "$0")"
PORT=${PORT:-8765}
python3 server.py --port "$PORT" "$@" &
SERVER=$!
trap 'kill -INT $SERVER; wait $SERVER' INT TERM
for _ in $(seq 20); do curl -s "http://127.0.0.1:$PORT/api/state" >/dev/null && break; sleep 0.2; done
URL="http://127.0.0.1:$PORT"
if command -v chromium >/dev/null; then chromium --app="$URL" >/dev/null 2>&1 &
elif command -v google-chrome >/dev/null; then google-chrome --app="$URL" >/dev/null 2>&1 &
else xdg-open "$URL" >/dev/null 2>&1 & fi
wait $SERVER
