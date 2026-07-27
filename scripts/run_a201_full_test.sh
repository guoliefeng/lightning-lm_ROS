#!/usr/bin/env bash
# A201 全量 bag 定位集成测试（131 段，~17min @ 1x）
set -euo pipefail

WS="${WS:-$HOME/proj/lightning-lm_ROS1_ws}"
PKG="$WS/src/lightning-lm_ROS"
BAG_DIR="/home/glf/dataDisk/hainan/yangpu/A201"
CONFIG="$PKG/config/yangpu_a201.yaml"
LOG="/tmp/a201_full_loc.log"
EVAL_BAG="/tmp/a201_full_eval.bag"
RATE="${RATE:-1.0}"

source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/noetic/setup.zsh
source "$WS/devel/setup.bash"

pkill -f "run_loc_online.*yangpu_a201" 2>/dev/null || true
pkill -f "rosbag record.*a201_full" 2>/dev/null || true
sleep 1

rosparam set use_sim_time true
rm -f "$LOG" "$EVAL_BAG"

echo "=== A201 full test start $(date) ===" | tee "$LOG"
echo "bags: $(ls "$BAG_DIR"/*.bag | wc -l), rate=$RATE" | tee -a "$LOG"

rosbag record -O "$EVAL_BAG" /tf /localization/ins __name:=a201_full_recorder &
REC_PID=$!
sleep 2

cd "$WS"
./devel/lib/lightning/run_loc_online \
  --config "$CONFIG" \
  --init_x 308.40 --init_y -150.60 --init_z -0.10 \
  --init_qx -0.001009 --init_qy 0.002793 \
  --init_qz 0.384632 --init_qw 0.923065 \
  >> "$LOG" 2>&1 &
LOC_PID=$!

echo "waiting for loc init..." | tee -a "$LOG"
for i in $(seq 1 30); do
  if grep -q "online loc node has been created" "$LOG" 2>/dev/null; then
    break
  fi
  sleep 1
done
sleep 2

echo "playing all A201 bags..." | tee -a "$LOG"
rosbag play --clock -r "$RATE" "$BAG_DIR"/*.bag 2>> "$LOG"
PLAY_EXIT=$?

echo "play exit=$PLAY_EXIT, stopping loc/recorder..." | tee -a "$LOG"
kill -INT "$LOC_PID" 2>/dev/null || true
sleep 3
kill -INT "$REC_PID" 2>/dev/null || true
sleep 2
kill -9 "$LOC_PID" "$REC_PID" 2>/dev/null || true

echo "=== A201 full test end $(date) play_exit=$PLAY_EXIT ===" | tee -a "$LOG"
ls -lh "$EVAL_BAG" 2>/dev/null | tee -a "$LOG" || true
