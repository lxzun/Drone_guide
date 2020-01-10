from e_drone.drone import *
from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep
from cv2 import imshow, imwrite, waitKey, destroyAllWindows, flip
from glob import glob
import numpy as np
import cv2

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))


############################## 이미지 처리 & 제어 파라미터 전달 ##############################

def image2cmd(BGRimage):
    yuvimage = cv2.cvtColor(BGRimage, cv2.COLOR_BGR2YUV)
    incrop = 150
    yuvimage = yuvimage[incrop:-150, 150:-150, :]
    center_idx = [80, 210]  # 세로, 가로

    u = yuvimage[:, :, 1]
    v = yuvimage[:, :, 2]

    biimage_yuv = np.zeros(np.shape(u))
    biimage_yuv[u > 135] = 200

    landbiimage_v = biimage_yuv.copy()
    a1 = u < 110
    a2 = v < 120
    a3 = np.logical_and(a1, a2)
    landbiimage_v[a3] = 255

    num_land = landbiimage_v > 250
    sum_land = sum(sum(num_land))

    global land_flag
    if sum_land > int(num_land.size) * 0.04:  # 착지 지점을 인식하는 threshold
        print('★ 착륙지점에 도착했습니다. ★')

        land_ix = np.where(255 == landbiimage_v)
        land_x = land_ix[0]
        land_y = land_ix[1]

        land_yy = int(np.mean(land_x))
        land_xx = int(np.mean(land_y))

        land_th = 35  # 착륙지점 threshold

        if (abs(land_xx - center_idx[1]) < land_th) and (abs(land_yy - center_idx[0]) < land_th):
            landing = 1
            print('############## 착륙합니다. ##############')
            return (landing, 0, 0, 0, 0)

        elif abs(land_xx - center_idx[1]) < land_th:
            if land_yy > center_idx[0]:
                print('착륙전 6시 방향이동')
                return (0, 0, -0.1, 0, 0.1)
            else:
                print('착륙전 12시 방향이동')
                return (0, 0, 0.1, 0, 0.1)

        elif abs(land_yy - center_idx[0]) < land_th:
            if land_xx > center_idx[1]:
                print('착륙전 3시 방향이동')
                return (0, 0, 0, -0.1, 0.1)
            else:
                print('착륙전 9시 방향이동')
                return (0, 0, 0, 0.1, 0.1)

    # 파라미터 초기화
    yaw_cmd = 0
    pitch_cmd = 0
    roll_cmd = 0
    vel = 0

    road_ix = np.where(200 == biimage_yuv)
    road_x = road_ix[0]
    road_y = road_ix[1]

    yy = int(np.mean(road_x))
    xx = int(np.mean(road_y))

    top_ix = np.where(road_x[0] == road_x)
    top_i = top_ix[0][-1]
    top_max = road_y[top_i]
    bottom_max = road_y[-1]

    # ROLL 제어 Part (좌우)
    if abs(center_idx[1] - xx) > 60:  # 좌우 threshold
        if xx > 220:
            roll_cmd = -0.1  ### 오른쪽 이동 정도
        else:
            roll_cmd = 0.1  ### 왼쪽 이동 정도
        vel = 0.2
        cucmd = 'roll control (+좌,-우) : ' + str(roll_cmd)

    # YAW 제어 Part (회전)
    elif abs(top_max - bottom_max) > 50:  # 각도 threshold
        roadangle = top_max - bottom_max
        yaw_cmd = int(roadangle / 3) * (-1)
        if yaw_cmd > 35:
            yaw_cmd = 35  ### 오른쪽 최대 회전 각도
        if yaw_cmd < -35:
            yaw_cmd = -35  ### 왼쪽 최대 회전 각도
        cucmd = 'yaw control (+오,-왼) : ' + str(yaw_cmd)

    # PITCH 제어 Part (앞뒤)
    else:
        pitch_cmd = 0.25  ### 앞으로 이동할 거리
        vel = 0.3  ### 속도
        cucmd = 'pitch control (앞) : ' + str(pitch_cmd)

    print(cucmd)
    return (0, yaw_cmd, pitch_cmd, roll_cmd, vel)


############################## Main Part ##############################

if __name__ == '__main__':

    i = 0
    drone = Drone()
    drone.open()

    print("★ 5초에 걸쳐 이륙을 시작합니다 ★")
    drone.sendTakeOff()
    sleep(5)

    # 이미지 읽어오기 시작합니다.
    img_list = glob('*.jpg')
    img_num = len(img_list)

    print("★ 자율비행을 시작합니다. ★")
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        i += 1  # 명령 카운트
        image = frame.array
        image = flip(image, 0)
        image = flip(image, 1)

        # 이미지를 저장하고 싶으면 주석해제
        # savename = 'first_{0}.jpg'.format(i)
        # cv2.imwrite(savename, image)

        rawCapture.truncate(0)

        print('************************* {0}번째 명령 *************************'.format(i))
        landing, yaw_cmd, pitch_cmd, roll_cmd, vel = image2cmd(image)
        end_time = time.time()
        print('이미지 처리시간 : ', end_time - start_time)

        if landing == 1:
            break

        drone.sendControlPosition(pitch_cmd, roll_cmd, 0, vel, yaw_cmd, 25)
        sleep(1.2)

    print("★ 자율비행을 마칩니다. 착륙합니다. ★")
    drone.sendLanding()