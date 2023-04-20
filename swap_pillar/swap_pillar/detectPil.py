import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Float64
from cv_bridge import CvBridge # sudo pip3 install cvbridge3 # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from ultralytics import YOLO
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
    self.cap = cv2.VideoCapture(0)
    self.model = YOLO("yolov8m.pt")

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    
    self.deltha_dis = []
    self.near_point = 0

  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    msg = Float64()
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    width = float(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = float(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    vector_cen = np.array([width/2, height/2])
    
    ret, frame = self.cap.read()

    if ret == True:
      results = self.model(frame, conf=0.8)
      
      annotated_frame = results[0].plot()
      center_coordinates = (int(vector_cen[0]), int(vector_cen[1]))
      
      for x, r in enumerate(results):
            for y, c in enumerate(r.boxes.cls):
                boxeswh = r.boxes.xywh.detach().cpu().numpy()
                # center_obj = (int(boxeswh[y][0]), int(boxeswh[y][1]))
                # self.deltha_dis.append(sqrt(pow(vector_cen[0] - boxeswh[y][0], 2) + pow(vector_cen[1] - boxeswh[y][1], 2)))
                center_obj = (int(boxeswh[y][0]), int(height/2))
                self.deltha_dis.append(sqrt(pow(vector_cen[0] - boxeswh[y][0], 2)))
                if y == len(r.boxes.cls) - 1:
                    self.near_point = min(self.deltha_dis)
                    msg.data = self.near_point
                    self.deltha_dis.clear()
                cv2.circle(annotated_frame, center_obj, 5, (0, 0, 0), -1)
                cv2.line(annotated_frame, center_coordinates, center_obj, (255, 0, 0), 2)
      cv2.circle(annotated_frame, center_coordinates, 5, (0, 0, 255), -1)
      # cv2.imshow("ROS2 publisher webcam",frame)
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message
      self.publisher_.publish(self.br.cv2_to_imgmsg(annotated_frame))
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