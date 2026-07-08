# 创建一个 Python 脚本
#cat > /tmp/save_image.py << 'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.sub = self.create_subscription(Image, '/hbmem_img', self.callback, 10)
        self.bridge = CvBridge()
        self.saved = False

    def callback(self, msg):
        if not self.saved:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imwrite('captured.jpg', cv_image)
            print("✅ 图片已保存到 /tmp/captured.jpg")
            self.saved = True
            rclpy.shutdown()

rclpy.init()
node = ImageSaver()
rclpy.spin(node)
EOF

# 运行脚本（需要摄像头节点正在运行）
#source /opt/tros/humble/setup.bash
#python3 /tmp/save_image.py