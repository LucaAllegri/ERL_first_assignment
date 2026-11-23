#!/usr/bin/env bash

workspace=$(cd "$(dirname "$0")" && pwd -P) || exit 1
runner="$workspace/run_assignment.sh"
cd "$workspace" || exit 1

# Gradient colors
C1='\e[31m'   # red
C2='\e[33m'   # yellow
C3='\e[32m'   # green
C4='\e[36m'   # cyan
C5='\e[34m'   # blue
RESET='\e[0m'

while true; do
    clear
    echo -e "${C1}                          ___           ___           ___                    ${RESET}"
    echo -e "${C1}                         /\  \         /\  \         /\  \                   ${RESET}"
    echo -e "${C1}                        /::\  \       /::\  \       /::\  \                  ${RESET}"
    echo -e "${C2}                       /:/\:\  \     /:/\:\  \     /:/\:\  \                 ${RESET}"
    echo -e "${C2}                      /::\ \:\  \   /:/  \:\  \   /:/  \:\  \                ${RESET}"
    echo -e "${C3}                     /:/\:\ \:\__\ /:/__/_\:\__\ /:/__/ \:\__\               ${RESET}"
    echo -e "${C3}                     \/_|::\/:/  / \:\  /\ \/__/ \:\  \ /:/  /               ${RESET}"
    echo -e "${C4}                        |:|::/  /   \:\ \:\__\    \:\  /:/  /                ${RESET}"
    echo -e "${C4}                        |:|\/__/     \:\/:/  /     \:\/:/  /                 ${RESET}"
    echo -e "${C5}                        |:|  |        \::/  /       \::/  /                  ${RESET}"
    echo -e "${C5}                         \|__|         \/__/         \/__/                   ${RESET}"

    echo ""
    echo "                           =============================="
    echo "                                   ASSIGNMENT TOOL"
    echo "                           =============================="
    echo ""
    echo "                                      S = Start"
    echo "                                      B = Build"
    echo "                                      T = Terminate"
    echo "                                      Q = Quit"
    echo ""
    echo "                           =============================="
    echo -e "${C2}                                            [ERL · Assignment 1]${RESET}"
    IFS= read -rsn 1 key || break
    case "$key" in
        [Ss]) bash "$runner" stop && bash "$runner" start ;;
        [Bb]) bash "$runner" build ;;
        [Tt]) bash "$runner" stop ;;
        [Qq]) bash "$runner" stop && exit 0 ;;
        *) continue ;;
    esac
    result=$?
    if (( result != 0 )) || [[ "$key" == [Bb] ]]; then
        printf '\nPremi un tasto per tornare al menu… '
        IFS= read -rsn 1 || break
    fi
done
