#!/usr/bin/env bash

COMPOSE_FILE="docker/docker-compose.yml"
GREEN='\033[1;32m'
RED='\033[1;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

MODE=""
SERVICE=""
CONTAINER_NAME=""

set_mode() {
    local selected="$1"

    case "$selected" in
        dev)
            MODE="dev"
            SERVICE="rby1_rviz_dev"
            CONTAINER_NAME="rby1_rviz_dev_container"
            ;;
        prod)
            MODE="prod"
            SERVICE="rby1_rviz_prod"
            CONTAINER_NAME="rby1_rviz_prod_container"
            ;;
        *)
            return 1
            ;;
    esac
}

select_mode() {
    while true; do
        clear
        echo "====================================="
        echo " RBY1 RViz Docker Mode"
        echo "====================================="
        echo "1. dev  - host source bind mount, runtime colcon build"
        echo "2. prod - build-time colcon build, source removed from image"
        echo "3. exit"
        echo "====================================="
        read -r -p "실행 모드를 선택하세요: " mode_choice

        case "$mode_choice" in
            1|dev)
                set_mode dev
                return 0
                ;;
            2|prod)
                set_mode prod
                return 0
                ;;
            3|exit|quit)
                exit 0
                ;;
            *)
                echo "잘못된 입력입니다."
                sleep 1
                ;;
        esac
    done
}

is_running() {
    [ -n "$(docker ps -q -f name=^/${CONTAINER_NAME}$)" ]
}

grant_xhost() {
    xhost +local:root > /dev/null 2>&1 || true
}

open_shell() {
    docker exec -it "$CONTAINER_NAME" bash -lc \
        "source /opt/ros/jazzy/setup.bash && source /app/install/setup.bash 2>/dev/null || true; cd /app; exec bash"
}

select_mode

while true; do
    clear

    if is_running; then
        STATUS="${GREEN}ON${NC}"
    else
        STATUS="${RED}OFF${NC}"
    fi

    echo "====================================="
    echo " RBY1 RViz Docker Manager"
    echo "====================================="
    echo -e "mode: ${YELLOW}${MODE}${NC}"
    echo -e "container: ${CONTAINER_NAME} (${STATUS})"
    echo "====================================="
    echo "1. Start selected mode (build & up)"
    echo "2. Open container shell"
    echo "3. Stop selected mode"
    echo "4. Switch mode"
    echo "5. Stop all modes"
    echo "6. Exit"
    echo "====================================="
    read -r -p "원하는 작업의 번호를 입력하세요: " choice

    case "$choice" in
        1)
            echo "Starting ${MODE} container..."
            grant_xhost
            docker compose -f "$COMPOSE_FILE" up -d --build "$SERVICE"
            echo ""
            echo "${MODE} container is running."
            sleep 2
            ;;
        2)
            if is_running; then
                grant_xhost
                open_shell
            else
                echo "Container is not running. Start it first."
                sleep 2
            fi
            ;;
        3)
            echo "Stopping ${MODE} container..."
            docker compose -f "$COMPOSE_FILE" stop "$SERVICE" > /dev/null 2>&1 || true
            docker compose -f "$COMPOSE_FILE" rm -f "$SERVICE" > /dev/null 2>&1 || true
            echo "Stopped ${MODE} container."
            sleep 2
            ;;
        4)
            select_mode
            ;;
        5)
            echo "Stopping all containers from this compose project..."
            docker compose -f "$COMPOSE_FILE" down
            echo "Stopped all modes."
            sleep 2
            ;;
        6)
            echo "Exiting."
            break
            ;;
        *)
            echo "잘못된 입력입니다."
            sleep 2
            ;;
    esac
done
