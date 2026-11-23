#!/usr/bin/env bash

workspace=$(cd "$(dirname "$0")" && pwd -P) || exit 1
runner="$workspace/run_assignment.sh"
repo="$workspace/src/ERL_first_assignment"
underlay=/opt/ros/jazzy/setup.bash
overlay="$workspace/install/erl_assignment1/local_setup.bash"
build_state="$workspace/build/erl_assignment1"
session=erl_assignment1
window="$session:dashboard"
dashboard=(tmux -L "$session")

cd "$workspace" || exit 1

# Gazebo model resource paths
while IFS= read -r model_dir; do
    export GZ_SIM_RESOURCE_PATH="$(dirname "$model_dir")${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
done < <(find "$repo" "$workspace/install/erl_assignment1"     -type d -name 'aruco_box_1' 2>/dev/null)


build() (
    mkdir -p "$build_state" || exit 1
    exec 9>"$build_state/.launcher-lock" || exit 1
    flock -n 9 || { printf 'Build già in corso.\n' >&2; exit 1; }
    touch "$build_state/.launcher-incomplete" || exit 1
    if [[ ! -f "$repo/erl1/package.xml" ]]; then
        printf 'Repository ERL non trovato in %s\n' "$repo" >&2
        exit 1
    fi
    source "$underlay" || exit 1
    colcon --log-base "$workspace/log/erl_assignment1" build \
        --base-paths "$repo" \
        --build-base "$workspace/build/erl_assignment1" \
        --install-base "$workspace/install/erl_assignment1" \
        --packages-up-to erl1 aruco_opencv robot_controller || exit $?
    rm -f -- "$build_state/.launcher-incomplete"
)

split() {
    "${dashboard[@]}" split-window "$1" -t "$2" -l "$3%" -c "$workspace" \
        -P -F '#{pane_id}' 'bash --noprofile --norc'
}

pane() {
    "${dashboard[@]}" select-pane -t "$1" -T "$2" &&
        "${dashboard[@]}" send-keys -t "$1" -l "source '$underlay' && source '$overlay' && exec $3" &&
        "${dashboard[@]}" send-keys -t "$1" Enter
}

create_dashboard() {
    local sim motion odom monitor commands debug runner_path
    printf -v runner_path '%q' "$runner"

    sim=$("${dashboard[@]}" new-session -d -s "$session" -n dashboard \
        -x 150 -y 48 -c "$workspace" -P -F '#{pane_id}' 'bash --noprofile --norc') &&
    motion=$(split -h "$sim" 50) &&
    odom=$(split -v "$sim" 67) &&
    commands=$(split -v "$odom" 50) &&
    monitor=$(split -v "$motion" 67) &&
    debug=$(split -v "$monitor" 50) &&
    "${dashboard[@]}" set-option -w -t "$window" pane-border-status top &&
    "${dashboard[@]}" set-option -w -t "$window" pane-border-format ' #{pane_title} ' &&
    "${dashboard[@]}" set-option -w -t "$window" remain-on-exit on &&
    "${dashboard[@]}" set-option -w -t "$window" pane-active-border-style fg=green &&
    "${dashboard[@]}" set-option -w -t "$window" pane-border-style fg=white &&
    "${dashboard[@]}" set-option -t "$session" status-style bg=black,fg=white &&
    "${dashboard[@]}" set-option -t "$session" mouse on &&
    "${dashboard[@]}" set-option -t "$session" @ros_panes "$sim" &&
    pane "$sim" 'ROBOT · GAZEBO / RVIZ' 'ros2 launch erl1 spawn_assignment.launch.py' &&
    pane "$motion" 'CONTROLLO' 'ros2 topic echo /cmd_vel geometry_msgs/msg/Twist' &&
    pane "$odom" 'ODOMETRIA' "bash $runner_path odometry" &&
    pane "$monitor" 'MONITOR ROS' "bash $runner_path monitor" &&
    pane "$commands" 'COMANDI' "bash $runner_path commands" &&
    "${dashboard[@]}" select-pane -t "$debug" -T 'DEBUG' &&
    "${dashboard[@]}" select-pane -t "$commands"
}

open_terminal() {
    if [[ -n "${DISPLAY:-}" ]] && command -v mate-terminal >/dev/null; then
        mate-terminal --maximize -- bash "$runner" attach
    elif [[ -n "${DISPLAY:-}" ]] && command -v gnome-terminal >/dev/null; then
        gnome-terminal --maximize -- bash "$runner" attach
    elif [[ -n "${DISPLAY:-}" ]] && command -v x-terminal-emulator >/dev/null; then
        x-terminal-emulator -e bash "$runner" attach &
    else
        exec "${dashboard[@]}" attach-session -t "$session"
    fi
}

start() {
    if [[ -e "$build_state/.launcher-incomplete" ]]; then
        printf 'Build in corso, interrotta o fallita. Completa B prima di avviare.\n' >&2
        return 1
    fi
    command -v tmux >/dev/null || {
        printf 'Serve tmux nel container.\n' >&2
        return 1
    }
    if "${dashboard[@]}" has-session -t "$session" 2>/dev/null; then
        if [[ $("${dashboard[@]}" display-message -pt "$session" '#{session_attached}') == 0 ]]; then
            open_terminal
        else
            printf 'La dashboard è già aperta.\n'
        fi
        return
    fi
    if [[ ! -r "$overlay" ]]; then
        printf 'Compila l’Assignment 1 con B prima di avviare.\n' >&2
        return 1
    fi
    source "$underlay" && source "$overlay" || return 1
    if ! create_dashboard; then
        stop
        return 1
    fi
    open_terminal || return 1
    focus_windows >/dev/null 2>&1 &
}

focus_windows() {
    [[ -n "${DISPLAY:-}" ]] && command -v wmctrl >/dev/null || return 0
    local attempt
    for ((attempt = 0; attempt < 15; attempt++)); do
        if wmctrl -a 'Gazebo Sim'; then
            if command -v xdotool >/dev/null; then
                xdotool search --onlyvisible --class '^rviz2$' windowminimize '%@'
            fi
            return 0
        fi
        sleep 1
    done
}

stop() {
    local pane_id attempt active
    local -a ros_panes
    "${dashboard[@]}" has-session -t "$session" 2>/dev/null || return 0
    read -ra ros_panes <<< "$("${dashboard[@]}" show-options -qv -t "$session" @ros_panes)"
    for pane_id in "${ros_panes[@]}"; do
        "${dashboard[@]}" send-keys -t "$pane_id" C-c 2>/dev/null || true
    done
    for ((attempt = 0; attempt < 30; attempt++)); do
        active=0
        for pane_id in "${ros_panes[@]}"; do
            if [[ $("${dashboard[@]}" display-message -pt "$pane_id" '#{pane_dead}' 2>/dev/null) == 0 ]]; then
                active=1
            fi
        done
        if (( !active )); then
            "${dashboard[@]}" kill-session -t "$session"
            return
        fi
        sleep 0.5
    done
    printf 'ROS sta ancora chiudendo. Riprova con T.\n' >&2
    return 1
}

snapshot() {
    printf '\nNODI ROS\n'
    ros2 node list
    printf '\nTOPIC ERL\n'
    ros2 topic list -t |
        grep -E '^/(odom|odometry/filtered|camera/image|aruco_detections|errors_topic|cmd_vel|marker_image)( |$)' ||
        true
}

monitor() {
    while true; do
        clear
        snapshot
        sleep 2
    done
}

odometry() {
    python3 - <<'PY'
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node("erl_dashboard_odometry")
latest = None


def receive(message):
    global latest
    latest = message


def render():
    print("\033[2J\033[HODOM STATE\n----------")
    if latest is None:
        print("In attesa di /odom…", flush=True)
        return

    pose = latest.pose.pose
    twist = latest.twist.twist
    q = pose.orientation
    yaw = math.atan2(
        2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
    )
    print(f"position.x = {pose.position.x: .3f}")
    print(f"position.y = {pose.position.y: .3f}")
    print(f"theta/yaw  = {yaw: .3f} rad\n")
    print(f"linear.x   = {twist.linear.x: .3f}")
    print(f"linear.y   = {twist.linear.y: .3f}")
    print(f"angular.z  = {twist.angular.z: .3f}", flush=True)


node.create_subscription(Odometry, "/odom", receive, qos_profile_sensor_data)
node.create_timer(1.0, render)
render()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
PY
}

gazebo_pause() {
    gz service -s /world/aruco_world/control \
        --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
        --timeout 3000 --req "pause: $1"
}

commands() {
    printf '%s\n' 'COMMAND PANEL' '' \
        'status            Nodi e topic ERL' \
        'marker            ID e posa del marker' \
        'error             Errori ArUco di distanza e allineamento' \
        'velocity          Ultimo cmd_vel ricevuto' \
        'pause / resume    Pausa o riprende Gazebo' \
        'help / exit       Guida o chiusura del pannello'
    local command
    while IFS= read -r -p 'erl_cmd> ' command; do
        case "$command" in
            status) snapshot ;;
            marker) timeout 5 ros2 topic echo --once /aruco_detections aruco_opencv_msgs/msg/ArucoDetection ;;
            error) timeout 5 ros2 topic echo --once /errors_topic robot_interfaces/msg/ErrorsMsg ;;
            velocity) timeout 5 ros2 topic echo --once /cmd_vel geometry_msgs/msg/Twist ;;
            pause) gazebo_pause true ;;
            resume) gazebo_pause false ;;
            help) printf '%s\n' 'status | marker | error | velocity | pause | resume | exit' ;;
            exit|quit) return ;;
            "") ;;
            *) printf 'Comando sconosciuto. Usa help.\n' ;;
        esac
        if [[ $? == 124 ]]; then
            printf 'Nessun messaggio ricevuto entro 5 secondi.\n'
        fi
    done
}

case "${1:-start}" in
    start) start ;;
    stop) stop ;;
    attach) "${dashboard[@]}" attach-session -t "$session" ;;
    build) build ;;
    commands) commands ;;
    monitor) monitor ;;
    odometry) odometry ;;
    *) printf 'Modo sconosciuto: %s\n' "$1" >&2; exit 2 ;;
esac
