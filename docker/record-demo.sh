#!/usr/bin/env bash
# Records a click-to-go demo of vision navigation: Gazebo GUI (left) and
# RViz (right) side by side, a "2D Goal Pose" dragged in RViz with the mouse,
# and ArduSub flying there with object avoidance. Run via `make record-demo`.
#
# By default everything is drawn on a private Xvfb display, so it works
# headless and never touches your desktop. Xvfb renders with the CPU, though,
# which is too slow for RTAB-Map VO: on a GPU machine use your own display
# (REC_DISPLAY=$DISPLAY) and leave the windows alone while it records. The goal is a real mouse drag (xdotool) with
# RViz's 2D Goal Pose tool, so what you see is exactly what a user does.
#
# Needs on the host: Xvfb, xdotool, xwininfo (x11-utils), ffmpeg.
#
# Settings (env):
#   GOAL="7 -1.5 90"     goal x y (Gazebo world, m) and heading (deg, ENU)
#                        default: just short of the SAUVC gate, straight
#                        through the orange flare, so the planner has to bend
#   OUT=recordings/vision_nav_demo.mp4
#   SPEEDUP=1            >1 speeds the final video up (slow software rendering)
#   TIMEOUT=900          seconds to wait for the goal before stopping anyway
#   VISION_SOURCE=vo     vo | gt  (as for bringup-vision)
#   VISION_DEPTH=1.2     goal depth (m)
#   GZ_SERVICE           compose service (as in the Makefile)
#   WORLD                world for Gazebo, RVIZ_WORLD for scene_markers
#   COMPOSE="docker compose"  e.g. add -f overrides
#   REC_DISPLAY=:99      an existing display (e.g. $DISPLAY) is used as-is
set -euo pipefail
cd "$(dirname "$0")/.."

GOAL=${GOAL:-"7 -1.5 90"}
OUT=${OUT:-recordings/vision_nav_demo.mp4}
SPEEDUP=${SPEEDUP:-1}
TIMEOUT=${TIMEOUT:-900}
VISION_SOURCE=${VISION_SOURCE:-vo}
VISION_DEPTH=${VISION_DEPTH:-1.2}
GZ_SERVICE=${GZ_SERVICE:-mira_sim}
WORLD=${WORLD:-/workspace/worlds/sauvc_competition.world}
RVIZ_WORLD=${RVIZ_WORLD:-$WORLD}
START_ENU=${START_ENU:-"7 -10"}
COMPOSE=${COMPOSE:-docker compose}
REC_DISPLAY=${REC_DISPLAY:-:99}
W=1920 H=1080 HALF=960
FPS=10

read -r GX GY GYAW <<<"$GOAL"
mkdir -p "$(dirname "$OUT")"
LOGDIR=$(mktemp -d /tmp/mira-record.XXXXXX)
RAW="$LOGDIR/raw.mp4"
log() { echo "[record-demo] $*"; }
in_sim() { $COMPOSE exec -T -e DISPLAY="$REC_DISPLAY" "$GZ_SERVICE" bash -c "$1"; }
bg_sim() {  # name, command; output to $LOGDIR/<name>.log
  in_sim "source /opt/ros/jazzy/setup.bash; $2" >"$LOGDIR/$1.log" 2>&1 &
}
stop_sim() { $COMPOSE exec -T "$GZ_SERVICE" pkill -f '/workspace/vision_nav/|gz sim|rviz2|parameter_bridge|static_transform_publisher|rgbd_odometry' 2>/dev/null || true; }

cleanup() {
  set +e
  [ -n "${FFMPEG_PID:-}" ] && kill -INT "$FFMPEG_PID" 2>/dev/null && wait "$FFMPEG_PID" 2>/dev/null
  stop_sim
  $COMPOSE stop -t 0 ardupilot-sitl-vision >/dev/null 2>&1
  [ -n "${XVFB_PID:-}" ] && kill "$XVFB_PID" 2>/dev/null
  log "logs: $LOGDIR"
}
trap cleanup EXIT

# --- virtual display ------------------------------------------------------
export DISPLAY=$REC_DISPLAY
if xdpyinfo >/dev/null 2>&1; then
  # An existing display, e.g. REC_DISPLAY=$DISPLAY: keeps GPU rendering
  # (Xvfb is software-only GL, too slow for VO). Records its top-left WxH.
  log "using existing display $REC_DISPLAY (top-left ${W}x${H} is recorded)"
  bash ./docker/x11-setup.sh
  xhost +local: >/dev/null 2>&1 || true
else
  log "Xvfb on $REC_DISPLAY (${W}x${H})"
  Xvfb "$REC_DISPLAY" -screen 0 ${W}x${H}x24 -ac +extension GLX +render -noreset >"$LOGDIR/xvfb.log" 2>&1 &
  XVFB_PID=$!
  for _ in $(seq 50); do xdpyinfo >/dev/null 2>&1 && break; sleep 0.2; done
  # compose bind-mounts the X cookie; Xvfb runs with -ac, so an empty one will do
  [ -e /tmp/.mira-sim.xauth ] || { touch /tmp/.mira-sim.xauth; chmod a+r /tmp/.mira-sim.xauth; }
fi

# --- simulator + vision nav stack (same pieces as `make bringup-vision`) ---
$COMPOSE up -d "$GZ_SERVICE" >/dev/null
stop_sim
$COMPOSE up -d ardupilot-sitl-vision >/dev/null
log "Gazebo: $WORLD"
bg_sim gazebo "source /tmp/gz-render-env.sh 2>/dev/null; exec gz sim -v3 -r --gui-config /workspace/vision_nav/record_gui.config $WORLD"
in_sim "until gz topic -l 2>/dev/null | grep -q /sim/front_camera/image; do sleep 1; done"
bg_sim bridge "exec bash /workspace/vision_nav/bridge.sh"
in_sim "source /opt/ros/jazzy/setup.bash; until ros2 topic list 2>/dev/null | grep -q /vision/image; do sleep 1; done"
[ "$VISION_SOURCE" = vo ] && bg_sim vo "exec bash /workspace/vision_nav/vo.sh"
bg_sim vision_bridge "exec python3 -u /workspace/vision_nav/vision_bridge.py --source $VISION_SOURCE --start-enu $START_ENU"
bg_sim obstacles "exec python3 -u /workspace/vision_nav/obstacle_distance.py"
bg_sim goal_bridge "exec python3 -u /workspace/vision_nav/goal_bridge.py --depth $VISION_DEPTH"
bg_sim scene "exec python3 -u /workspace/vision_nav/scene_markers.py $RVIZ_WORLD"
bg_sim rviz "exec rviz2 -d /workspace/vision_nav/record.rviz --ros-args -p use_sim_time:=true"

wait_log() {  # file, pattern, what
  log "waiting for $3 ..."
  until grep -q "$2" "$LOGDIR/$1.log" 2>/dev/null; do sleep 2; done
}
wait_log vision_bridge "Aligned" "ArduSub heading alignment"
wait_log goal_bridge "Connected" "goal_bridge"

# --- lay the windows out side by side ---------------------------------------
find_win() { for _ in $(seq 60); do w=$(xdotool search --onlyvisible --name "$1" | head -1); [ -n "$w" ] && { echo "$w"; return; }; sleep 1; done; return 1; }
GZ_WIN=$(find_win "Gazebo Sim")
RVIZ_WIN=$(find_win "RViz")
xdotool windowsize "$GZ_WIN" $HALF $H windowmove "$GZ_WIN" 0 0
xdotool windowsize "$RVIZ_WIN" $HALF $H windowmove "$RVIZ_WIN" $HALF 0
sleep 5

# RViz's 3D view is the largest native child window; with the TopDownOrtho
# view in record.rviz its centre is (X, Y) and Scale is pixels per metre.
read -r PX PY HX HY RX RY RW RH < <(python3 - "$RVIZ_WIN" "$GX" "$GY" "$GYAW" <<'PY'
import math, re, subprocess, sys
win, gx, gy, yaw = sys.argv[1], *map(float, sys.argv[2:])
cfg = open('src/vision_nav/record.rviz').read()
view = cfg[cfg.index('TopDownOrtho'):]
vx, vy, scale = (float(re.search(rf'\n\s+{k}: (\S+)', view).group(1)) for k in ('X', 'Y', 'Scale'))
# RViz's 3D view is the largest native child window of the RViz window
best = None
for line in subprocess.run(['xwininfo', '-tree', '-id', win], capture_output=True, text=True).stdout.splitlines():
    m = re.search(r'(\d+)x(\d+)[+-]-?\d+[+-]-?\d+\s+\+(-?\d+)\+(-?\d+)\s*$', line)
    if m:
        w, h, x, y = map(int, m.groups())
        if best is None or w * h > best[0] * best[1]:
            best = (w, h, x, y)
w, h, x, y = best
# TopDownOrtho: view centre (X, Y) at the panel centre, Scale pixels per metre
px, py = x + w / 2 + (gx - vx) * scale, y + h / 2 - (gy - vy) * scale
hx, hy = px + 60 * math.cos(math.radians(yaw)), py - 60 * math.sin(math.radians(yaw))
print(*(round(v) for v in (px, py, hx, hy, x, y, w, h)))
PY
)
log "RViz 3D view ${RW}x${RH}+${RX}+${RY}; goal ($GX, $GY) -> pixel ($PX, $PY)"

# --- record -----------------------------------------------------------------
log "recording -> $RAW"
ffmpeg -loglevel error -y -f x11grab -framerate $FPS -video_size ${W}x${H} -i "$REC_DISPLAY" \
  -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p "$RAW" &
FFMPEG_PID=$!
sleep 6

# 2D Goal Pose: pick the tool, then press at the goal and drag towards the heading
xdotool mousemove $((RX + RW / 2)) $((RY + RH / 2)) sleep 1
xdotool windowfocus "$RVIZ_WIN" key g sleep 1  # "g" = 2D Goal Pose
xdotool mousemove --sync "$PX" "$PY" sleep 0.8 mousedown 1 sleep 0.4
for i in 1 2 3 4 5 6; do
  xdotool mousemove --sync $((PX + (HX - PX) * i / 6)) $((PY + (HY - PY) * i / 6)) sleep 0.15
done
xdotool sleep 0.5 mouseup 1
xdotool mousemove $((HALF - 40)) $((H - 40))  # park the cursor out of the way
wait_log goal_bridge "Goal:" "goal_bridge to take the goal"
grep "Goal:" "$LOGDIR/goal_bridge.log" | tail -1

START=$SECONDS
until grep -q "Goal reached" "$LOGDIR/goal_bridge.log"; do
  (( SECONDS - START > TIMEOUT )) && { log "timeout waiting for the goal"; break; }
  sleep 5
done
grep "Goal reached" "$LOGDIR/goal_bridge.log" | tail -1 || true
sleep 20  # let it turn to the goal heading
kill -INT "$FFMPEG_PID"; wait "$FFMPEG_PID" || true; FFMPEG_PID=

if [ "$SPEEDUP" != 1 ]; then
  ffmpeg -loglevel error -y -i "$RAW" -vf "setpts=PTS/$SPEEDUP" -r 30 -c:v libx264 -preset medium -crf 22 -pix_fmt yuv420p -an "$OUT"
else
  cp "$RAW" "$OUT"
fi
log "done: $OUT"
