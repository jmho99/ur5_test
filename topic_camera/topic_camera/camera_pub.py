import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import time
import threading

hsv = 0
lower_blue1 = 0
upper_blue1 = 0
lower_blue2 = 0
upper_blue2 = 0
lower_blue3 = 0
upper_blue3 = 0
a = 0
result = []

def nothing(x):
    pass

class CameraPublisher(Node):
    def __init__(self,capture):
        super().__init__('camera_publisher')

        # Publisher 생성 (QoS 설정 포함)
        self.coordinate_publisher = self.create_publisher(
            Float64,
            'input_number',
            10)
        
        self.completion_subscriber = self.create_subscription(
            Bool,
            'task_completed',
            self.completion_callback,
            10)
        
        self.waiting_for_completion = False
        # 시작 전 잠시 대기 (노드 초기화 시간 확보)
        time.sleep(1.0)

        cv.namedWindow('img_color')
        cv.namedWindow('img_result')
        cv.createTrackbar('threshold', 'img_result', 0, 255, nothing)
        cv.setTrackbarPos('threshold', 'img_result', 30)

        self.cap = capture
        if not self.cap.isOpened():
            self.get_logger().error("비디오 캡처를 열 수 없습니다.")
            return

        # 첫 번째 메시지 발행
        self.send_next_number()

        # 상태 체크 타이머
        self.create_timer(2.0, self.check_status)

    def completion_callback(self, msg):
        self.get_logger().info(f'완료 메시지 수신: {msg.data}')
        self.waiting_for_completion = msg.data
        

    def send_next_number(self):
    
        while(True):
            result = []
            #img_color = cv.imread('2.jpg')
            ret,img_color = self.cap.read()
            height, width = img_color.shape[:2]
            img_color = cv.resize(img_color, (width, height), interpolation=cv.INTER_AREA)

            # 원본 영상을 HSV 영상으로 변환합니다.
            img_hsv = cv.cvtColor(img_color, cv.COLOR_BGR2HSV)
            img_gray = cv.cvtColor(img_color, cv.COLOR_BGR2GRAY)
            a, binary_img = cv.threshold(img_gray,150,255,cv.THRESH_BINARY)
            # 범위 값으로 HSV 이미지에서 마스크를 생성합니다.

            img_mask1 = cv.inRange(img_hsv, lower_blue1, upper_blue1)
            img_mask2 = cv.inRange(img_hsv, lower_blue2, upper_blue2)
            img_mask3 = cv.inRange(img_hsv, lower_blue3, upper_blue3)
            img_mask = img_mask1 | img_mask2 | img_mask3

            kernel = np.ones((11,11), np.uint8)
            img_mask = cv.morphologyEx(img_mask, cv.MORPH_OPEN, kernel)
            img_mask = cv.morphologyEx(img_mask, cv.MORPH_CLOSE, kernel)

            # 마스크 이미지로 원본 이미지에서 범위값에 해당되는 영상 부분을 획득합니다.
            img_result = cv.bitwise_and(img_color, img_color, mask=img_mask)


            numOfLabels, img_label, stats, centroids = cv.connectedComponentsWithStats(img_mask)

            for idx, centroid in enumerate(centroids):
                if stats[idx][0] == 0 and stats[idx][1] == 0:
                    continue

                if np.any(np.isnan(centroid)):
                    continue

                x,y,width,height,area = stats[idx]
                centerX,centerY = int(centroid[0]), int(centroid[1])
                print(centerX, centerY)
                msg = float()
                msg.data = centerX
                self.coordinate_publisher.publish(msg)
                result.append([idx, centerX, centerY])


                if area > 50:
                    cv.circle(img_color, (centerX, centerY), 10, (0,0,255), 10)
                    cv.rectangle(img_color, (x,y), (x+width,y+height), (0,0,255))

            cv.imshow('img_color', img_color)
        #cv.imshow('img_mask', img_mask)
        #cv.imshow('img_result', img_result)

def main(args=None):
    rclpy.init(args=args)
    cap = cv.VideoCapture("rtmp://192.168.0.163:1935/live")
    publisher = CameraPublisher(cap)
    try:
        
        rclpy.spin(publisher)
    except Exception as e:
        print(f"에러 발생: {e}")
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
