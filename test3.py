<<<<<<< HEAD
## cyz 2025.7.31 读取螺母的位姿，控制机械臂两阶段移动到目标位姿（使用四元数和相对变换）
## 第一段 先移动到螺母上方
## 第二段 下落
import sys
sys.path.append('./VREP_api')
import cv2
import time
import numpy as np
import sim  # 请确保你已正确安装 remoteApi 的 Python 封装
from scipy.spatial.transform import Rotation as R, Slerp

# 连接到 CoppeliaSim
print('Connecting to CoppeliaSim...')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote API server')

    sim.simxSynchronous(clientID, True)
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)

    # 摄像头（可选显示图像）
    _, visionSensorHandle = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)
    sim.simxGetVisionSensorImage(clientID, visionSensorHandle, 0, sim.simx_opmode_streaming)
    time.sleep(0.1)

    # 获取 target 初始姿态
    _, targetHandle = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking)
    sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_streaming)
    sim.simxGetObjectQuaternion(clientID, targetHandle, -1, sim.simx_opmode_streaming)
    time.sleep(0.1)

    _, p_init = sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_buffer)
    _, q_init = sim.simxGetObjectQuaternion(clientID, targetHandle, -1, sim.simx_opmode_buffer)
    q_init_rot = R.from_quat(q_init)

    # 获取 Nut 位姿
    _, nutHandle = sim.simxGetObjectHandle(clientID, 'Nut', sim.simx_opmode_blocking)
    sim.simxGetObjectPosition(clientID, nutHandle, -1, sim.simx_opmode_streaming)
    sim.simxGetObjectOrientation(clientID, nutHandle, -1, sim.simx_opmode_streaming)
    time.sleep(0.1)

    _, nut_pos = sim.simxGetObjectPosition(clientID, nutHandle, -1, sim.simx_opmode_buffer)
    _, nut_euler = sim.simxGetObjectOrientation(clientID, nutHandle, -1, sim.simx_opmode_buffer)

    # 构建 Nut 的齐次矩阵
    nut_rot = R.from_euler('xyz', nut_euler)
    T_nut = np.eye(4)
    T_nut[0:3, 0:3] = nut_rot.as_matrix()
    T_nut[0:3, 3] = nut_pos

    # -------------------- 第一阶段目标位姿 --------------------
    # 先绕 Y 轴旋转 -90°
    offset_rot1 = R.from_euler('y', -np.pi/2)
    T_offset1 = np.eye(4)
    T_offset1[0:3, 0:3] = offset_rot1.as_matrix()

    # 再沿其自身Z轴方向后退0.2m
    T_zback = np.eye(4)
    T_zback[2, 3] = -0.08

    T_target1 = T_nut @ T_offset1 @ T_zback
    p1 = T_target1[0:3, 3].tolist()
    q1_rot = R.from_matrix(T_target1[0:3, 0:3])
    q1 = q1_rot.as_quat().tolist()

    # 插值移动到第一目标
    key_rots1 = R.from_quat([q_init, q1])
    slerp1 = Slerp([0, 1], key_rots1)

    num = 150
    for i in range(1, num + 1):
        alpha = i / num
        cur_p = [p_init[j] + alpha * (p1[j] - p_init[j]) for j in range(3)]
        cur_q = slerp1([alpha])[0].as_quat().tolist()

        sim.simxSetObjectPosition(clientID, targetHandle, -1, cur_p, sim.simx_opmode_oneshot)
        sim.simxSetObjectQuaternion(clientID, targetHandle, -1, cur_q, sim.simx_opmode_oneshot)
        sim.simxSynchronousTrigger(clientID)
        time.sleep(0.2)

    # -------------------- 第二阶段目标位姿 --------------------
    # 在 T_target1 坐标系下，沿 Z 轴正方向移动 0.08m
    T_zforward = np.eye(4)
    T_zforward[2, 3] = 0.08

    T_target2 = T_target1 @ T_zforward
    p2 = T_target2[0:3, 3].tolist()
    q2_rot = R.from_matrix(T_target2[0:3, 0:3])
    q2 = q2_rot.as_quat().tolist()

    # 插值第二段
    key_rots2 = R.from_quat([q1, q2])
    slerp2 = Slerp([0, 1], key_rots2)

    for i in range(1, num + 1):
        alpha = i / num
        cur_p = [p1[j] + alpha * (p2[j] - p1[j]) for j in range(3)]
        cur_q = slerp2([alpha])[0].as_quat().tolist()

        sim.simxSetObjectPosition(clientID, targetHandle, -1, cur_p, sim.simx_opmode_oneshot)
        sim.simxSetObjectQuaternion(clientID, targetHandle, -1, cur_q, sim.simx_opmode_oneshot)
        sim.simxSynchronousTrigger(clientID)
        time.sleep(0.02)

    # -------------------- 结束 --------------------
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    sim.simxFinish(clientID)
    print('Simulation finished.')

else:
    print('Failed connecting to remote API server')
=======
## cyz 2025.7.31 读取螺母的位姿，控制机械臂两阶段移动到目标位姿（使用四元数和相对变换）
## 第一段 先移动到螺母上方
## 第二段 下落
import sys
sys.path.append('./VREP_api')
import cv2
import time
import numpy as np
import sim  # 请确保你已正确安装 remoteApi 的 Python 封装
from scipy.spatial.transform import Rotation as R, Slerp

# 连接到 CoppeliaSim
print('Connecting to CoppeliaSim...')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote API server')

    sim.simxSynchronous(clientID, True)
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)

    # 摄像头（可选显示图像）
    _, visionSensorHandle = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)
    sim.simxGetVisionSensorImage(clientID, visionSensorHandle, 0, sim.simx_opmode_streaming)
    time.sleep(0.1)

    # 获取 target 初始姿态
    _, targetHandle = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking)
    sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_streaming)
    sim.simxGetObjectQuaternion(clientID, targetHandle, -1, sim.simx_opmode_streaming)
    time.sleep(0.1)

    _, p_init = sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_buffer)
    _, q_init = sim.simxGetObjectQuaternion(clientID, targetHandle, -1, sim.simx_opmode_buffer)
    q_init_rot = R.from_quat(q_init)

    # 获取 Nut 位姿
    _, nutHandle = sim.simxGetObjectHandle(clientID, 'Nut', sim.simx_opmode_blocking)
    sim.simxGetObjectPosition(clientID, nutHandle, -1, sim.simx_opmode_streaming)
    sim.simxGetObjectOrientation(clientID, nutHandle, -1, sim.simx_opmode_streaming)
    time.sleep(0.1)

    _, nut_pos = sim.simxGetObjectPosition(clientID, nutHandle, -1, sim.simx_opmode_buffer)
    _, nut_euler = sim.simxGetObjectOrientation(clientID, nutHandle, -1, sim.simx_opmode_buffer)

    # 构建 Nut 的齐次矩阵
    nut_rot = R.from_euler('xyz', nut_euler)
    T_nut = np.eye(4)
    T_nut[0:3, 0:3] = nut_rot.as_matrix()
    T_nut[0:3, 3] = nut_pos

    # -------------------- 第一阶段目标位姿 --------------------
    # 先绕 Y 轴旋转 -90°
    offset_rot1 = R.from_euler('y', -np.pi/2)
    T_offset1 = np.eye(4)
    T_offset1[0:3, 0:3] = offset_rot1.as_matrix()

    # 再沿其自身Z轴方向后退0.2m
    T_zback = np.eye(4)
    T_zback[2, 3] = -0.08

    T_target1 = T_nut @ T_offset1 @ T_zback
    p1 = T_target1[0:3, 3].tolist()
    q1_rot = R.from_matrix(T_target1[0:3, 0:3])
    q1 = q1_rot.as_quat().tolist()

    # 插值移动到第一目标
    key_rots1 = R.from_quat([q_init, q1])
    slerp1 = Slerp([0, 1], key_rots1)

    num = 150
    for i in range(1, num + 1):
        alpha = i / num
        cur_p = [p_init[j] + alpha * (p1[j] - p_init[j]) for j in range(3)]
        cur_q = slerp1([alpha])[0].as_quat().tolist()

        sim.simxSetObjectPosition(clientID, targetHandle, -1, cur_p, sim.simx_opmode_oneshot)
        sim.simxSetObjectQuaternion(clientID, targetHandle, -1, cur_q, sim.simx_opmode_oneshot)
        sim.simxSynchronousTrigger(clientID)
        time.sleep(0.2)

    # -------------------- 第二阶段目标位姿 --------------------
    # 在 T_target1 坐标系下，沿 Z 轴正方向移动 0.08m
    T_zforward = np.eye(4)
    T_zforward[2, 3] = 0.08

    T_target2 = T_target1 @ T_zforward
    p2 = T_target2[0:3, 3].tolist()
    q2_rot = R.from_matrix(T_target2[0:3, 0:3])
    q2 = q2_rot.as_quat().tolist()

    # 插值第二段
    key_rots2 = R.from_quat([q1, q2])
    slerp2 = Slerp([0, 1], key_rots2)

    for i in range(1, num + 1):
        alpha = i / num
        cur_p = [p1[j] + alpha * (p2[j] - p1[j]) for j in range(3)]
        cur_q = slerp2([alpha])[0].as_quat().tolist()

        sim.simxSetObjectPosition(clientID, targetHandle, -1, cur_p, sim.simx_opmode_oneshot)
        sim.simxSetObjectQuaternion(clientID, targetHandle, -1, cur_q, sim.simx_opmode_oneshot)
        sim.simxSynchronousTrigger(clientID)
        time.sleep(0.02)

    # -------------------- 结束 --------------------
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    sim.simxFinish(clientID)
    print('Simulation finished.')

else:
    print('Failed connecting to remote API server')
>>>>>>> 82147d2 (first commit)
