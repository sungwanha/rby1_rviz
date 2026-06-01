import socket
import subprocess
import time

# 설정: 퀘스트 앱이 접속할 포트들을 리스트로 관리
REVERSE_PORTS = [5000, 9091] 

def get_pc_ip():
    """현재 PC의 로컬 IP를 가져옵니다 (NTP 주입용)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 1))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def run_adb(command_list):
    """ADB 명령어를 실행하고 성공 여부를 반환합니다."""
    try:
        subprocess.run(['adb'] + command_list, check=True, capture_output=True, text=True)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False

def full_setup():
    """포트 포워딩과 시간 동기화를 순차적으로 수행합니다."""
    pc_ip = get_pc_ip()
    print(f"\n[작업 시작] 🛠️  PC IP: {pc_ip}")

    # 1. ADB Reverse 설정 (포트 포워딩)
    print("[1/2] ADB Reverse 설정 중...")
    for port in REVERSE_PORTS:
        success = run_adb(['reverse', f'tcp:{port}', f'tcp:{port}'])
        if success:
            print(f"   ✅ Port {port} reverse 완료")
        else:
            print(f"   ❌ Port {port} reverse 실패")

    # 2. 시간 동기화 설정
    print("[2/2] 시간 동기화(NTP) 설정 중...")
    run_adb(['shell', 'settings', 'put', 'global', 'ntp_server', pc_ip])
    run_adb(['shell', 'settings', 'put', 'global', 'auto_time', '0'])
    time.sleep(0.5)
    run_adb(['shell', 'settings', 'put', 'global', 'auto_time', '1'])
    
    try:
        q_time = subprocess.check_output(['adb', 'shell', 'date'], text=True).strip()
        print(f"   ✅ 동기화 완료! 현재 퀘스트 시간: {q_time}")
    except:
        print("   ⚠️  동기화 명령 전송 완료 (시간 확인 불가)")

def monitor_loop():
    """연결 상태를 감시하며 끊겼다 다시 붙을 때 모든 세팅을 다시 합니다."""
    was_connected = False
    print("\n=========================================")
    print("  퀘스트 자동화 시스템 (Reverse + NTP) 가동 중  ")
    print("=========================================")

    while True:
        is_connected = run_adb(['get-state'])

        if is_connected and not was_connected:
            print("\n[EVENT] 🔌 기기 연결 감지! 전체 세팅을 시작합니다.")
            full_setup()
            was_connected = True
            print("\n[STATUS] 📡 세팅 완료. 상시 감시 모드...")

        elif not is_connected and was_connected:
            print("\n[WARN] ❌ 기기 연결 해제됨! 다시 연결될 때까지 대기합니다.")
            was_connected = False
        
        # CPU 부하 방지를 위해 연결 대기 중엔 2초, 연결 유지 중엔 5초 간격 체크
        interval = 5 if is_connected else 2
        time.sleep(interval)

if __name__ == "__main__":
    try:
        monitor_loop()
    except KeyboardInterrupt:
        print("\n[EXIT] 자동화 프로그램을 종료합니다.")