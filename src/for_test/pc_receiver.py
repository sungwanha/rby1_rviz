import socket
import struct
import numpy as np
import cv2
import subprocess
import time

def setup_adb_reverse():
    """메타퀘스트가 연결될 때까지 포트 포워딩을 무한 시도합니다."""
    print("\n[ADB] 기기 연결 및 포트 포워딩 상태를 점검합니다...")
    
    while True:
        try:
            subprocess.run(
                ['adb', 'reverse', 'tcp:5000', 'tcp:5000'],
                check=True, capture_output=True, text=True
            )
            
            subprocess.run(
                ['adb', 'reverse', 'tcp:9091', 'tcp:9091'],
                check=True, capture_output=True, text=True
            )
            print("[ADB] ✅ 포트 설정 완료: Video(5000), ROS(9091)")
            break  
        except (subprocess.CalledProcessError, FileNotFoundError):
            time.sleep(2)

def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: 
            return None
        buf += newbuf
        count -= len(newbuf)
    return buf

# TCP 서버 설정
HOST = '127.0.0.1' 
PORT = 5000
server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_sock.bind((HOST, PORT))
server_sock.listen(1)

HEADER_FORMAT = '<3I 32f' 
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

while True:
    setup_adb_reverse()
    
    print(f"[{PORT}] TCP 서버 대기 중... 퀘스트 앱을 실행하세요.")
    
    try:
        conn, addr = server_sock.accept()
        print(f"\n[연결됨] 퀘스트 스트리밍이 시작되었습니다!")
        
        while True:
            header_data = recvall(conn, HEADER_SIZE)
            if not header_data: break
            
            unpacked = struct.unpack(HEADER_FORMAT, header_data)
            magic, frame_no, jpeg_size = unpacked[0:3]
            
            if magic != 0x55444458: break

            payload_data = recvall(conn, jpeg_size)
            if not payload_data: break

            np_arr = np.frombuffer(payload_data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            poses = unpacked[3:35] 
        
            hmd_pos = poses[0:3]   # XYZ
            hmd_rot = poses[3:6]   # RPY
            
            r_tracked = poses[6]
            r_pos = poses[7:10]    # XYZ
            r_rot = poses[10:13]   # RPY
        
            l_tracked = poses[13]
            l_pos = poses[14:17]   # XYZ
            l_rot = poses[17:20]   # RPY

            print(f"\r[HMD] Pos: {hmd_pos[2]:.3f}m | Rot: {hmd_rot[1]:.1f}deg", end="")
            
            if img is not None:
                font = cv2.FONT_HERSHEY_SIMPLEX
                f_scale = 0.55
                thick = 2
                
                color_hmd = (0, 255, 0)
                cv2.putText(img, f"HMD Pos: {poses[0]:.3f}, {poses[1]:.3f}, {poses[2]:.3f}", (10, 30), font, f_scale, color_hmd, thick)
                cv2.putText(img, f"HMD Rot: R:{poses[3]:.1f}, P:{poses[4]:.1f}, Y:{poses[5]:.1f}", (10, 55), font, f_scale, color_hmd, thick)
                
                if r_tracked > 0.5:
                    color_r = (255, 255, 0)
                    cv2.putText(img, f"R-Pos: {poses[7]:.3f}, {poses[8]:.3f}, {poses[9]:.3f}", (10, 90), font, f_scale, color_r, thick)
                    cv2.putText(img, f"R-Rot: R:{poses[10]:.1f}, P:{poses[11]:.1f}, Y:{poses[12]:.1f}", (10, 115), font, f_scale, color_r, thick)
                else:
                    cv2.putText(img, "R-Ctrl: Not Tracked", (10, 90), font, f_scale, (100, 100, 100), 1)

                if l_tracked > 0.5:
                    color_l = (255, 0, 255)
                    cv2.putText(img, f"L-Pos: {poses[14]:.3f}, {poses[15]:.3f}, {poses[16]:.3f}", (10, 150), font, f_scale, color_l, thick)
                    cv2.putText(img, f"L-Rot: R:{poses[17]:.1f}, P:{poses[18]:.1f}, Y:{poses[19]:.1f}", (10, 175), font, f_scale, color_l, thick)
                else:
                    cv2.putText(img, "L-Ctrl: Not Tracked", (10, 150), font, f_scale, (100, 100, 100), 1)

                cv2.imshow('Quest Camera Stream (USB TCP)', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    conn.close()
                    cv2.destroyAllWindows()
                    exit()

    except Exception as e:
        print(f"스트리밍 에러: {e}")
    finally:
        if 'conn' in locals(): conn.close()
        cv2.destroyAllWindows()
        print("[연결 종료] 스트리밍이 끊겼습니다. 복구를 시도합니다...")