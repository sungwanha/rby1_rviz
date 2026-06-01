#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import serial.tools.list_ports

class TriggerBridge(Node):
    def __init__(self):
        super().__init__('trigger_bridge_node')
        self.left_pub = self.create_publisher(Joy, '/simdd/trigger_left', 10)
        self.right_pub = self.create_publisher(Joy, '/simdd/trigger_right', 10)
        
        self.connections = {}
        self.baud_rate = 115200
        
        # ESP32-C3 고유 번호 (lsusb 결과 기반)
        self.TARGET_VID = 0x303A
        self.TARGET_PID = 0x1001
        
        # 포트 스캔 타이머 (2초), 데이터 읽기 타이머 (100Hz)
        self.create_timer(2.0, self.update_connections)
        self.create_timer(0.01, self.read_serial_data)
        
        self.get_logger().info("--- Strict Hardware Filter Mode ---")
        self.get_logger().info(f"Targeting: VID={hex(self.TARGET_VID)}, PID={hex(self.TARGET_PID)}")

    def update_connections(self):
        # [핵심] VID와 PID가 정확히 일치하는 포트만 필터링해서 리스트업
        all_ports = serial.tools.list_ports.comports()
        active_ports = [
            p.device for p in all_ports 
            if p.vid == self.TARGET_VID and p.pid == self.TARGET_PID
        ]
        
        # 연결 해제 처리 (주머니에서 제거)
        closed_ports = [p for p in self.connections if p not in active_ports]
        for port in closed_ports:
            self.get_logger().warn(f"ESP32-C3 removed from {port}")
            self.connections[port].close()
            del self.connections[port]
            
        # 신규 연결 처리 (주머니에 추가)
        for port in active_ports:
            if port not in self.connections:
                try:
                    new_ser = serial.Serial(port, self.baud_rate, timeout=0.1)
                    self.connections[port] = new_ser
                    self.get_logger().info(f"SUCCESS: ESP32-C3 Connected on {port}")
                except Exception as e:
                    self.get_logger().error(f"Error opening {port}: {e}")

        # 테스트용 로그: 아무것도 없을 때
        if not self.connections and not active_ports:
            # 매번 찍으면 시끄러우니 2초마다 장치 찾는 중임을 알림
            pass 

    def read_serial_data(self):
        # 연결된 보드가 있을 때만 작동
        for port, ser in list(self.connections.items()):
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    # self.get_logger().info(f"RECVD: [{line}]")
                    if not line: continue
                    
                    data = line.split(',')
                    if len(data) >= 2:
                        board_id = data[0].strip()
                        try:
                            norm_value = float(data[-1].strip())
                            msg = Joy()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = "trigger_link"
                            msg.axes = [norm_value]
                            
                            if "LEFT" in board_id:
                                self.left_pub.publish(msg)
                            elif "RIGHT" in board_id:
                                self.right_pub.publish(msg)
                        except: pass
            except Exception:
                if port in self.connections:
                    del self.connections[port]

def main(args=None):
    rclpy.init(args=args)
    node = TriggerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()