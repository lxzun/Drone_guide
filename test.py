import numpy as np
import cv2
import math

route_r = cv2.imread('/home/lxz/workdir/Drone/Drone_guide/img/image_r.jpg')
route_l = cv2.imread('/home/lxz/workdir/Drone/Drone_guide/img/image_l.jpg')
route_a = cv2.imread('/home/lxz/workdir/Drone/Drone_guide/img/image_a.jpg')

for img in [route_r, route_l, route_a]:
    img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    center_idx = [int(img.shape[0]/2), int(img.shape[1]/2)]  # 세로, 가로

    y = img[:, :, 0]
    u = img[:, :, 1]
    v = img[:, :, 2]

    th, biimage_yuv = cv2.threshold(v, 100, 255, cv2.THRESH_BINARY_INV)
    cv2.imwrite('/home/lxz/workdir/Drone/Drone_guide/img/biimg.jpg', biimage_yuv)


    # 파라미터 초기화
    yaw_cmd = 0
    pitch_cmd = 0
    roll_cmd = 0
    vel = 0

    road_ix = np.where(255 == biimage_yuv)
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
#     return (0, yaw_cmd, pitch_cmd, roll_cmd, vel)