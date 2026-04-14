#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy  # 조이스틱/트리거 전용 표준 메시지
import serial
import serial.tools.list_ports

class TriggerBridge(Node):
    def __init__(self):
        super().__init__('trigger_bridge_node')
        
        # 메시지 타입을 Joy로 변경
        self.left_pub = self.create_publisher(Joy, '/simdd/trigger_left', 10)
        self.right_pub = self.create_publisher(Joy, '/simdd/trigger_right', 10)
        
        self.connections = {}
        self.baud_rate = 115200
        
        self.create_timer(2.0, self.update_connections)
        self.create_timer(0.01, self.read_serial_data)
        
        self.get_logger().info("ROS 2 Trigger Bridge Started (Joy/Standard Mode)")

    def update_connections(self):
        active_ports = [p.device for p in serial.tools.list_ports.comports()]
        closed_ports = [p for p in self.connections if p not in active_ports]
        for port in closed_ports:
            self.connections[port].close()
            del self.connections[port]
            
        for port in active_ports:
            if port not in self.connections:
                try:
                    new_ser = serial.Serial(port, self.baud_rate, timeout=0.1)
                    self.connections[port] = new_ser
                    self.get_logger().info(f"Connected to {port}")
                except:
                    pass

    def read_serial_data(self):
        for port, ser in list(self.connections.items()):
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    if not line: continue
                    
                    data = line.split(',')
                    if len(data) >= 2:
                        board_id = data[0].strip()
                        try:
                            norm_value = float(data[-1].strip())
                            
                            # Joy 메시지 생성
                            msg = Joy()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = "trigger_link"
                            
                            # 트리거 값은 '축(axes)' 데이터의 첫 번째 요소로 할당
                            msg.axes = [norm_value]
                            msg.buttons = [] # 버튼은 비워둠

                            if "LEFT" in board_id:
                                self.left_pub.publish(msg)
                            elif "RIGHT" in board_id:
                                self.right_pub.publish(msg)
                                
                        except ValueError:
                            continue
            except Exception:
                if port in self.connections:
                    del self.connections[port]

def main(args=None):
    rclpy.init(args=args)
    node = TriggerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()