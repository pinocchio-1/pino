import sys
sys.path.append('./VREP_api')
import cv2
import time
import numpy as np
import sim  # 请确保你已正确安装 remoteApi 的 Python 封装

# 连接到 CoppeliaSim
print('Connecting to CoppeliaSim...')
sim.simxFinish(-1)  # 清除以前的连接
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote API server')

    # 开启同步模式
    sim.simxSynchronous(clientID, True)
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)

    # 获取摄像头句柄
    errCode, visionSensorHandle  = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)
    # 初始化图像信息
    _, _, _ = sim.simxGetVisionSensorImage(clientID, visionSensorHandle, 0, sim.simx_opmode_streaming)
    time.sleep(0.1)  # 等待摄像头数据准备好

    # 获取目标句柄（如 MATLAB 中的 LINKID）
    errCode, targetHandle = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking)

    # 初始化位置和姿态获取（buffer 模式前需要先调用一次 streaming）
    sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_streaming)
    sim.simxGetObjectOrientation(clientID, targetHandle, -1, sim.simx_opmode_streaming)
    time.sleep(0.1)

    _, p_init = sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_buffer)
    _, r_init = sim.simxGetObjectOrientation(clientID, targetHandle, -1, sim.simx_opmode_buffer)

    # 目标位置和姿态（相对世界坐标系）
    p_end = [p_init[0] - 0.1, p_init[1], p_init[2] - 0.1]
    r_end = [r_init[0] + np.deg2rad(50), r_init[1], r_init[2]]

    # 进行插值移动
    num = 100
    for i in range(1, num + 1):
        alpha = i / num
        cur_p = [p_init[j] + alpha * (p_end[j] - p_init[j]) for j in range(3)]
        cur_r = [r_init[j] + alpha * (r_end[j] - r_init[j]) for j in range(3)]

        sim.simxSetObjectPosition(clientID, targetHandle, -1, cur_p, sim.simx_opmode_oneshot)
        sim.simxSetObjectOrientation(clientID, targetHandle, -1, cur_r, sim.simx_opmode_oneshot)

        sim.simxSynchronousTrigger(clientID)
        time.sleep(0.02)  # 可选：节省 CPU 负载

        # 获取摄像头图像
        errCode, resolution, image = sim.simxGetVisionSensorImage(clientID, visionSensorHandle, 0, sim.simx_opmode_buffer)
        if errCode == sim.simx_return_ok:
            image = np.array(image, dtype=np.int8)
            image = image.astype(np.int16)
            image = np.clip(image + 128, 0, 255).astype(np.uint8)

            image = image.reshape((resolution[1], resolution[0], 3))
            image = cv2.flip(image, 0)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # 可选增强亮度
            # image = cv2.convertScaleAbs(image, alpha=1.5, beta=30)

            cv2.imshow("Vision Sensor View", image)
            cv2.waitKey(1)

    # 停止仿真
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    sim.simxFinish(clientID)
    print('Simulation finished.')

else:
    print('Failed connecting to remote API server')
