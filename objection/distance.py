import numpy as np
import pyzed.sl as sl
import warnings


def pix_distance(height, width, xmin, ymin, mat):
    data = np.full((height, width), np.nan)
    for i in range(height):
        for j in range(width):
            a = mat.get_value(xmin + i, ymin + j)
            if a[0] == sl.ERROR_CODE.SUCCESS and np.isfinite(a[1]):
                data[i, j] = a[1]
    warnings.simplefilter("ignore", category=RuntimeWarning)
    distances = np.nanmin(data)
    return distances


# 整个检测框的距离
def distance(mat, xmin, ymin, xmax, ymax):
    height = ymax - ymin
    width = xmax - xmin
    distances = pix_distance(height, width, xmin, ymin, mat)
    return distances


# 人距离车辆的真实距离
def abs_distance(mat, xmin, ymin, xmax, ymax):
    height = int((ymax - ymin) / 2)
    width = xmax - xmin
    h = 2.35  # 车辆摄像机距离地面的高度
    d2 = pix_distance(height, width, xmin, ymin + height, mat)
    if d2 > h:
        d = np.sqrt(np.square(d2) - np.square(h))
    else:
        d = np.nan
    return d


# 行人高度，代码暂时有问题
def human_height(mat, xmin, ymin, xmax, ymax):
    height = int((ymax - ymin) / 2)
    width = xmax - xmin
    h = 2.35  # 车辆摄像机距离地面的高度
    d1 = pix_distance(height, width, xmin, ymin, mat)
    d2 = pix_distance(height, width, xmin, ymin + height, mat)
    delta = np.sqrt(np.square(h) - np.square(d2) + np.square(d1))
    if delta > 0:
        d1 = h + delta if h + delta > 0 else 0
        d2 = h - delta if h + delta > 0 else 0
        d = d1 if d1 < d2 else d2
    else:
        d = np.nan
    return d if d > 0 else 0


# 定义检测框的信息类
class Value:
    def __init__(self, xmin, xmax, ymin, ymax, name, confidence):
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.name = name
        self.confidence = confidence


# 获取ZED相机的图像数据，删除第3维，并将其copy
class Image:
    def __init__(self, image_mat):
        image_ocv = image_mat.get_data()
        image = np.delete(image_ocv, -1, axis=2)
        self.img = image.copy()
