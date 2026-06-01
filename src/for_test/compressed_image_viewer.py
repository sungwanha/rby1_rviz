import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import cv2
import numpy as np

class CompressedImageViewer(Node):
    def __init__(self):
        super().__init__('compressed_image_viewer_node')
        
        # Publisher의 QoS 프로파일에 맞춰 설정 (통신 실패 방지)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 토픽 구독 설정
        self.subscription = self.create_subscription(
            CompressedImage,
            '/simdd/head_cam/compressed',
            self.image_callback,
            qos_profile
        )
        self.get_logger().info('Waiting for compressed images on /simdd/head_cam/compressed...')

    def image_callback(self, msg):
        try:
            # 1. byte array 형태의 데이터를 numpy 배열로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            
            # 2. numpy 배열을 OpenCV 이미지(Mat)로 디코딩
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                # 3. OpenCV 윈도우에 이미지 출력
                cv2.imshow("Head Cam Viewer", cv_image)
                
                # 1ms 대기 (GUI 이벤트를 처리하기 위해 필수)
                cv2.waitKey(1)
            else:
                self.get_logger().warn('Failed to decode the compressed image.')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageViewer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        # 종료 시 리소스 정리
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()