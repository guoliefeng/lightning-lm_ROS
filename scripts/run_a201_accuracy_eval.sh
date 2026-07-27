#!/usr/bin/env bash
# A201 定位精度评测：常速播放 bag，录制 TF+INS，evo 对比 INS 参考
set -euo pipefail

WS="${WS:-$HOME/proj/lightning-lm_ROS1_ws}"
PKG="$WS/src/lightning-lm_ROS"
BAG_DIR="/home/glf/dataDisk/hainan/yangpu/A201"
CONFIG="$PKG/config/yangpu_a201.yaml"
RATE="${RATE:-1.0}"
TAG="${TAG:-a201_acc}"
OUT_DIR="${OUT_DIR:-/tmp/${TAG}}"
LOG="${OUT_DIR}/loc.log"
EVAL_BAG="${OUT_DIR}/eval.bag"
MAX_BAGS="${MAX_BAGS:-0}"   # 0 = 全部 131 段

source /opt/ros/noetic/setup.bash 2>/dev/null || true
source "$WS/devel/setup.bash"

mkdir -p "$OUT_DIR"
pkill -f "run_loc_online.*yangpu_a201" 2>/dev/null || true
pkill -f "rosbag record.*${TAG}" 2>/dev/null || true
sleep 1

rosparam set use_sim_time true
rm -f "$LOG" "$EVAL_BAG"

echo "=== A201 accuracy eval start $(date) tag=$TAG rate=$RATE ===" | tee "$LOG"

rosbag record -O "$EVAL_BAG" /tf /localization/ins __name:=${TAG}_recorder &
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
for i in $(seq 1 40); do
  if grep -q "online loc node has been created" "$LOG" 2>/dev/null; then
    break
  fi
  sleep 1
done
sleep 2

mapfile -t BAGS < <(ls "$BAG_DIR"/*.bag | sort)
if [[ "$MAX_BAGS" -gt 0 ]]; then
  BAGS=("${BAGS[@]:0:$MAX_BAGS}")
fi
echo "playing ${#BAGS[@]} bags @ ${RATE}x ..." | tee -a "$LOG"
rosbag play --clock -r "$RATE" "${BAGS[@]}" 2>> "$LOG"
PLAY_EXIT=$?

echo "play exit=$PLAY_EXIT, stopping..." | tee -a "$LOG"
kill -INT "$LOC_PID" 2>/dev/null || true
sleep 3
kill -INT "$REC_PID" 2>/dev/null || true
sleep 2
kill -9 "$LOC_PID" "$REC_PID" 2>/dev/null || true

EST_TUM="${OUT_DIR}/est.tum"
REF_TUM="${OUT_DIR}/ref_ins.tum"
EVO_DIR="${OUT_DIR}/evo"
mkdir -p "$EVO_DIR"

python3 "$PKG/scripts/export_bag_to_tum.py" "$EVAL_BAG" \
  --est-out "$EST_TUM" --ref-out "$REF_TUM" --config "$CONFIG" | tee -a "$LOG"

python3 "$PKG/scripts/eval_loc_accuracy.py" "$EVAL_BAG" --config "$CONFIG" --align | tee "${OUT_DIR}/custom_metrics.txt"

if command -v evo_ape >/dev/null; then
  echo "--- evo_ape (SE3, align) ---" | tee "${OUT_DIR}/evo_ape.txt"
  evo_ape tum "$REF_TUM" "$EST_TUM" -a \
    --save_results "${EVO_DIR}/ape.zip" 2>&1 | tee -a "${OUT_DIR}/evo_ape.txt" || true

  echo "--- evo_rpe (trans, delta=1m) ---" | tee "${OUT_DIR}/evo_rpe.txt"
  evo_rpe tum "$REF_TUM" "$EST_TUM" -a --delta 1 --delta_unit m \
    --save_results "${EVO_DIR}/rpe.zip" 2>&1 | tee -a "${OUT_DIR}/evo_rpe.txt" || true
else
  echo "evo not found, skip evo_ape" | tee -a "$LOG"
fi

echo "=== done $(date) outputs in $OUT_DIR ===" | tee -a "$LOG"
ls -lh "$EVAL_BAG" "$EST_TUM" "$REF_TUM" 2>/dev/null | tee -a "$LOG"
