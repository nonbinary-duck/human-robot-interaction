import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from std_msgs.msg import Int32MultiArray,Float32MultiArray
##OTHER library imports
import cv2
import numpy as np
import imutils

class ImageProcessing(Node):
    def __init__(self):
        
        super().__init__('ImagePRO')
        #subscribing to get the image to process
        self.sub = self.create_subscription(Image, '/camera/color/image_raw', self.Cam_callback, 10)
        #making an instance of open cv
        self.br = CvBridge()
    def Cam_callback(self,data):
        print("bruh")

        # Initializing the HOG person
        # detector
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        # Reading the Image
        image = self.br.imgmsg_to_cv2(data,desired_encoding='bgr8')
        
        image = imutils.resize(image,width =min(600,image.shape[1]))
        # Detecting all the regions in the 
        # Image that has a pedestrians inside it
        (regions, _) = hog.detectMultiScale(image, winStride=(4, 4),padding=(4, 4),scale=1.05)
        
        # Drawing the regions in the Image
        for (x, y, w, h) in regions:
            cv2.rectangle(image, (x, y), 
                        (x + w, y + h), 
                        (0, 0, 255), 2)
        
        # Showing the output Image
        cv2.imshow("Image", image)
        cv2.waitKey(1)
        
        # cv2.destroyAllWindows()



def main():
    print('Starting image processing')

    rclpy.init()

    IMGpro= ImageProcessing()

    rclpy.spin(IMGpro)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    IMGpro.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()