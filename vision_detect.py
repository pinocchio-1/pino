# vision_detect.py
import numpy as np
import cv2

def convert_vrep_image(image, resolution):
    """
    将 V-REP 获取的一维图像列表转换为 OpenCV 格式的 BGR 图像
    """
    w, h = resolution
    img = np.array(image, dtype=np.int8).astype(np.uint8)
    img = img.reshape((h, w, 3))
    img = cv2.flip(img,
                   0)  # V-REP 图像上下颠倒，需翻转
    return img


def detect_circles(image_bgr, show_result=False, save_result=False):
    """
    检测图像中的圆形目标，返回圆心和半径的列表，并在图像上进行标注
    """
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    gray = cv2.medianBlur(gray, 5)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20,
                               param1=50, param2=15, minRadius=30, maxRadius=60)

    result = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for (x, y, r) in circles[0, :]:
            result.append((x, y, r))
            # 画圆轮廓
            cv2.circle(image_bgr, (x, y), r, (0, 255, 0), 2)
            # 画圆心
            cv2.circle(image_bgr, (x, y), 2, (0, 0, 255), 3)

    if save_result:
        cv2.imwrite("detected_circles.png", image_bgr)

    # 直接返回标注过的图像和结果
    return image_bgr, result


# 将图像坐标转换为世界坐标
import numpy as np

def pixel_to_camera(u, v, resolution, fov_x_deg=60, fov_y_deg=45, z_camera=0.6):
    """
    将图像像素坐标 (u,v) 转换为相机坐标系下的 (x,y,z)
    假设相机垂直向下，z_camera 是相机离地面的高度（单位：米）
    """
    width, height = resolution
    fov_x = np.radians(fov_x_deg)
    fov_y = np.radians(fov_y_deg)

    fx = width / (2 * np.tan(fov_x / 2))
    fy = height / (2 * np.tan(fov_y / 2))

    cx = width / 2
    cy = height / 2

    # 像素 → 相机坐标（忽略透视畸变）
    x = (u - cx) * z_camera / fx
    y = (v - cy) * z_camera / fy
    z = z_camera  # 相机向下看，z 为常数高度

    return x, y, z

# 检测六边形
def detect_hexagons(image_bgr, show_result=False, save_result=False):
    """
    检测图像中的六边形轮廓（例如螺母），在图像中标注结果并返回中心坐标列表
    """
    output_img = image_bgr.copy()
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)

    # 自适应阈值提取边缘
    thresh = cv2.adaptiveThreshold(gray, 255,
                                   cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV,
                                   blockSize=11, C=3)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 80:
            continue  # 过滤小噪声

        # 多边形拟合
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        num_vertices = len(approx)

        if 5 <= num_vertices <= 7 and cv2.isContourConvex(approx):
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                result.append((cX, cY))

                cv2.drawContours(output_img, [approx], -1, (0, 255, 0), 2)
                cv2.circle(output_img, (cX, cY), 4, (0, 0, 255), -1)
                cv2.putText(output_img, f"Hex6? ({num_vertices})", (cX-25, cY-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    if show_result:
        cv2.imshow("Detected Hexagons", output_img)
        cv2.waitKey(1)

    if save_result:
        cv2.imwrite("detected_hexagons.png", output_img)

    return output_img, result


# 检测六边形外观+圆形内孔 的形状
def detect_nut_shape(image_bgr, show_result=False, save_result=False):
    """
    检测具有“六边形外轮廓 + 圆形内孔”结构的螺母，返回中心点坐标列表
    """
    output_img = image_bgr.copy()
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)

    thresh = cv2.adaptiveThreshold(gray, 255,
                                   cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV,
                                   blockSize=11, C=3)

    # 获取轮廓及层级信息
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    result = []

    if hierarchy is not None:
        hierarchy = hierarchy[0]  # 层级格式：[Next, Prev, First_Child, Parent]

        for i, cnt in enumerate(contours):
            # 判断是否是外轮廓（存在子轮廓）
            has_child = hierarchy[i][2] != -1
            if not has_child:
                continue

            area_outer = cv2.contourArea(cnt)
            if area_outer < 100:
                continue

            # 多边形逼近外轮廓
            epsilon = 0.03 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            if 5 <= len(approx) <= 7 and cv2.isContourConvex(approx):
                # 查找其子轮廓
                child_index = hierarchy[i][2]
                cnt_child = contours[child_index]
                area_inner = cv2.contourArea(cnt_child)

                # 检查子轮廓是否近似圆（使用圆形拟合 + 面积限制）
                (x, y), radius = cv2.minEnclosingCircle(cnt_child)
                circle_area = np.pi * radius * radius
                ratio = area_inner / circle_area

                if 0.6 < ratio < 1.1 and area_inner > 20:
                    # 满足“六边形+孔”结构
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        result.append((cX, cY))

                        cv2.drawContours(output_img, [approx], -1, (0, 255, 0), 2)
                        cv2.circle(output_img, (cX, cY), 4, (0, 0, 255), -1)
                        cv2.putText(output_img, "Nut", (cX - 20, cY - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    if show_result:
        cv2.imshow("Detected Nuts", output_img)
        cv2.waitKey(1)

    if save_result:
        cv2.imwrite("detected_nuts.png", output_img)

    return output_img, result
