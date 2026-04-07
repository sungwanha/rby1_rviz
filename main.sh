#!/bin/bash

# --- 설정 구간 ---
# docker ps에서 확인된 정확한 이름을 입력하세요.
CONTAINER_NAME="rby1_rviz_container"
GREEN='\033[1;32m'
RED='\033[1;31m'
NC='\033[0m'

while true; do
    clear 

    # 정확한 이름 매칭을 위해 정규표현식(^, $) 사용
    if [ "$(docker ps -q -f name=^/${CONTAINER_NAME}$)" ]; then
        STATUS="${GREEN}🟢 (현재 켜짐)${NC}"
    else
        STATUS="${RED}🔴 (현재 꺼짐)${NC}"
    fi

    echo "====================================="
    echo " 🐳 UDD Project 도커 관리 매니저 🐳 "
    echo "====================================="
    echo "1. 도커 환경 켜기 (Build & Start)"
    echo -e "2. 도커 터미널 접속 ${STATUS}"
    echo "3. 도커 환경 끄기 (Stop & Remove)"
    echo "4. 종료"
    echo "====================================="
    read -p "원하는 작업의 번호를 입력하세요: " choice

    case $choice in
        1)
            echo "🚀 빌드 파일 정리 및 컨테이너 시작..."
            # [중요] 호스트(Humble)와 컨테이너(Jazzy)의 경로 충돌 방지를 위해 기존 빌드 폴더 삭제
            rm -rf build/ install/ log/
            
            # GUI 권한 부여
            xhost +local:root > /dev/null 2>&1
            
            # 빌드 및 백그라운드 실행
            docker compose up -d --build
            echo ""
            echo "✅ 컨테이너가 성공적으로 켜졌습니다!"
            sleep 2
            ;;
        2)
            if [ "$(docker ps -q -f name=^/${CONTAINER_NAME}$)" ]; then
                echo "💻 도커 내부로 접속합니다..."
                xhost +local:root > /dev/null 2>&1
                # 접속 시 자동으로 ROS 2 환경을 로드하도록 설정
                docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/jazzy/setup.bash && source /app/install/setup.bash 2>/dev/null || true; bash"
            else
                echo -e "${RED}⚠️ 오류: 컨테이너가 아직 꺼져있습니다! 1번을 먼저 눌러주세요.${NC}"
                sleep 2
            fi
            ;;
        3)
            echo "🛑 도커 컨테이너를 중지합니다..."
            # [해결책] compose down이 실패할 경우를 대비해 이름으로 직접 중지 및 삭제
            docker stop $CONTAINER_NAME > /dev/null 2>&1
            docker rm -f $CONTAINER_NAME > /dev/null 2>&1
            
            # 혹시 모를 네트워크/볼륨 정리를 위해 down도 병행
            docker compose down > /dev/null 2>&1
            
            echo "✅ 완전히 종료되었습니다!"
            sleep 2
            ;;
        4)
            echo "👋 매니저를 종료합니다."
            break
            ;;
        *)
            echo "❌ 잘못된 입력입니다. 1~4 사이의 숫자가 아닙니다."
            sleep 2
            ;;
    esac
done