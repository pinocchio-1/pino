import sys
sys.path.append('./VREP_api')
import cv2
import time
import numpy as np
import sim  # 请确保你已正确安装 remoteApi 的 Python 封装

# ============ 图像识别函数：识别图像中的螺母中心位置 ============ #
def detect_nut_center(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    c = max(contours, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy)


# ============ 控制策略：图像偏差 → target 位置调整 ============ #
def visual_servo_to_center(cx, cy, resolution, current_pos):
    img_cx = resolution[0] // 2
    img_cy = resolution[1] // 2
    dx = cx - img_cx
    dy = cy - img_cy

    # 像素误差映射到世界坐标，需根据实际情况微调 scale
    scale = 0.0005
    new_pos = current_pos.copy()
    new_pos[0] -= dx * scale  # x 轴控制左右
    new_pos[1] += dy * scale  # y 轴控制上下
    return new_pos


# ============ 新增主函数：视觉伺服控制 target 对准螺母 ============ #
def visual_servo_main(clientID):
    print("Starting visual servo...")

    # 获取 Vision Sensor 和 target 句柄
    _, visionSensorHandle = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)
    _, targetHandle = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking)

    # 初始化图像流和位置缓存
    sim.simxGetVisionSensorImage(clientID, visionSensorHandle, 0, sim.simx_opmode_streaming)
    sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_streaming)
    time.sleep(0.2)

    _, cur_p = sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_buffer)

    for step in range(200):
        errCode, resolution, image = sim.simxGetVisionSensorImage(clientID, visionSensorHandle, 0, sim.simx_opmode_buffer)
        if errCode != sim.simx_return_ok:
            sim.simxSynchronousTrigger(clientID)
            continue

        image = np.array(image, dtype=np.int8).astype(np.int16)
        image = np.clip(image + 128, 0, 255).astype(np.uint8)
        image = image.reshape((resolution[1], resolution[0], 3))
        image = cv2.flip(image, 0)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        nut_center = detect_nut_center(image)
        if nut_center is not None:
            cx, cy = nut_center
            cur_p = visual_servo_to_center(cx, cy, resolution, cur_p)
            sim.simxSetObjectPosition(clientID, targetHandle, -1, cur_p, sim.simx_opmode_oneshot)
            cv2.circle(image, nut_center, 5, (0, 255, 0), -1)

        cv2.imshow("Vision Servo", image)
        cv2.waitKey(1)
        sim.simxSynchronousTrigger(clientID)
        time.sleep(0.05)


# ============ 主入口，可选择调用不同功能 ============ #
if __name__ == '__main__':
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print('Connected to CoppeliaSim')

        sim.simxSynchronous(clientID, True)
        sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)

        # 选择调用：main / visual_servo_main
        # main()
        visual_servo_main(clientID)

        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
        sim.simxFinish(clientID)
        print('Simulation finished.')

    else:
        print('Failed to connect')
