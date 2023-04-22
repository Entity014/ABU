import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Float64
import cv2 # OpenCV library
from cv_bridge import CvBridge # sudo pip3 install cvbridge3 # Package to convert between ROS and OpenCV Images
import numpy as np
from math import sqrt, pow

class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('Camera')

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'image', 10)
    self.publisher2_ = self.create_publisher(Float64, 'distance_xy', 10)

    # We will publish a message every 0.1 seconds
    timer_period = 0.01  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    # self.GSTREAMER_PIPELINE = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=720, height=490, format=(string)NV12, framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, width=720, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'
    # self.cap = cv2.VideoCapture(self.GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)

    self.cap = cv2.VideoCapture(0)

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    
    self.deltha_dis = []
    self.near_point = 0
    self.temp = 0
    self.lower_range = np.array([19, 89, 88])
    self.upper_range = np.array([28, 255, 255])

  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    num = 0
    msg = Float64()
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    width = float(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = float(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    vector_cen = np.array([width/2, height/2])
    
    ret, frame = self.cap.read()

    if ret == True:
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, self.lower_range, self.upper_range)
      res = cv2.bitwise_and(frame,frame, mask= mask)
      contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      center_coordinates = (int(vector_cen[0]), int(vector_cen[1]))
      for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        area = cv2.contourArea(cnt)

        if area > 500:
            x,y,w,h = cv2.boundingRect(cnt)
            if len(approx) > 5:
                num +=1
                center_obj = (int(x + (w/2)), int(height/2))
                self.deltha_dis.append(sqrt(pow(vector_cen[0] - (x + (w/2)), 2)))
                if num > self.temp:
                    self.temp = num
                elif num == 1:
                    self.temp = num
                    self.near_point = min(self.deltha_dis)
                    msg.data = self.near_point
                    self.deltha_dis.clear()
                
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # cv2.drawContours(frame,[cnt],0,(255,255,0),-1)
                cv2.putText(frame,'Circle',(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),2)
                cv2.circle(frame, center_obj, 5, (0, 0, 0), -1)
                cv2.line(frame, center_coordinates, center_obj, (255, 0, 0), 2)

      cv2.circle(frame, center_coordinates, 5, (0, 0, 255), -1)
      # cv2.imshow("ROS2 publisher webcam",frame)
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
      self.publisher2_.publish(msg)
      if cv2.waitKey(1) & 0xFF == ord('q'):
        #closing all open windows 
        cv2.destroyAllWindows()
        exit() 

    # Display the message on the console
    self.get_logger().info('Publishing video frame')

def main(args=None):
  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  image_publisher = ImagePublisher()

  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
