# test_vision2.py
import sys
sys.path.append('./VREP_api')
import sim
import time
import cv2
from vision_detect import *
import numpy as np

print('Connecting to VREP...')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:
    print('Connected to VREP')
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

    err, visionSensorHandle = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)
    sim.simxGetVisionSensorImage(clientID, visionSensorHandle, 0, sim.simx_opmode_streaming)
    time.sleep(0.5)

    while True:
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, visionSensorHandle, 0, sim.simx_opmode_buffer)
        if err == sim.simx_return_ok:
            print("图像获取成功")
            print(f"图像分辨率: {resolution}")
            print(f"前20个像素值: {image[:20]}")
            img_bgr = convert_vrep_image(image, resolution)

            img_annotated, nut_centers = detect_nut_shape(img_bgr, show_result=True, save_result=False)

            # ✅ Python显示Vrep中的视觉传感器窗口
            cv2.imshow("VREP Vision Sensor:", img_annotated)
            cv2.waitKey(1)

            if nut_centers:
                u, v = nut_centers[0]
                x_cam, y_cam, z_cam = pixel_to_camera(u, v, resolution, z_camera=0.2)
                print(f"相机坐标下的螺母目标: X={x_cam:.3f}, Y={y_cam:.3f}, Z={z_cam:.3f}")

        else:
            print("图像获取失败，等待图像流初始化")

        time.sleep(0.1)

else:
    print('Failed to connect to VREP')
