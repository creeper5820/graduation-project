#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONTAINER="diffphys_ros2"
NO_BUILD=true

for arg in "$@"; do
    case "$arg" in
        --rebuild|-b) NO_BUILD=false ;;
    esac
done

# ── 1. Build image & start container ────────────────────────────
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    if [ "$NO_BUILD" = true ]; then
        echo "Starting container (no build)..."
        docker compose -f "$SCRIPT_DIR/docker-compose.yaml" up -d
    else
        echo "Building image & starting container..."
        docker compose -f "$SCRIPT_DIR/docker-compose.yaml" up -d --build
    fi
    sleep 3
fi

# ── 2. Clean up residual processes ──────────────────────────────
docker exec "${CONTAINER}" bash -c \
    'pkill -f "ros2|ego_planner|diffphys|random_forest|pcl_render|foxglove|validator|so3_|component_container|quadrotor_simulator" 2>/dev/null' || true
sleep 1

# ── 3. Kill old tmux session ────────────────────────────────────
tmux kill-session -t sim 2>/dev/null || true

# Force tmux to use bash instead of fish
tmux start-server 2>/dev/null || true
tmux set -g default-shell /bin/bash 2>/dev/null || true

SETUP="source /opt/ros/jazzy/setup.bash && source /workspace/simulation/diffphys_ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"

# ── 4. Pane 0: foxglove bridge ─────────────────────────────────
tmux new-session -d -s sim -n launch \; \
    send-keys -t sim:launch \
        "docker exec ${CONTAINER} bash -c '${SETUP} && exec ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765'" Enter

# ── 5. Pane 1: simulation ──────────────────────────────────────
tmux split-window -h -t sim:launch \; \
    send-keys -t sim:launch \
        "docker exec ${CONTAINER} bash -c '${SETUP} && exec ros2 launch diffphys_local_planner single_run_diffphys.launch.py'" Enter

# ── 6. Pane 2: goal + validator (runs a script inside container) ──
tmux split-window -v -t sim:launch \; \
    send-keys -t sim:launch \
        "bash -c 'docker exec ${CONTAINER} bash /workspace/simulation/goal_and_validate.sh'" Enter

echo ""
echo "=== Launch done ==="
echo "Attach:  tmux attach -t sim"
echo "Stop:    tmux kill-session -t sim"
echo "Foxglove: ws://localhost:8765"
echo ""
