import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class PassthroughViewer(Node):
    def __init__(self):
        super().__init__('passthrough_viewer')
        
        # 1. 구독 설정: 토픽 이름 '/simdd/passthrough_camera', 타입 CompressedImage
        self.subscription = self.create_subscription(
            CompressedImage,
            '/simdd/passthrough_camera',
            self.listener_callback,
            10) # QoS 설정 (심도 10)
        
        self.get_logger().info('Quest Passthrough Viewer Node Started')

    def listener_callback(self, msg):
        try:
            # 2. 메시지의 바이트 데이터를 넘파이 배열로 변환
            # 만약 송신측에서 Base64로 보냈더라도, ROS2 브릿지를 거쳤다면 
            # msg.data는 이미 바이너리(bytes) 상태입니다.
            np_arr = np.frombuffer(msg.data, np.uint8)

            # 3. JPEG 압축 해제 (바이너리 -> OpenCV BGR 이미지)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is not None:
                # 4. 화면 출력
                cv2.imshow("Quest Passthrough Camera", cv_image)
                
                # 'q' 키를 누르면 창 닫기 (실제로는 waitKey가 있어야 창이 업데이트됨)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info('Closing viewer...')
            else:
                self.get_logger().warn('Failed to decode image. Check if the format is truly JPEG.')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    viewer = PassthroughViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()