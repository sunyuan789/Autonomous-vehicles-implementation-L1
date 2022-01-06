import numpy as np
import torch

# 加载本地YoloV5模型 or 在线模型
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
model = torch.hub.load('yolov5', 'custom', path='weights/best_epochs50.pt', source='local')


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


# 输出检测目标物
class Objection(Image):
    def __init__(self, image_mat):
        super().__init__(image_mat)
        result = model(self.img, size=640)
        self.results = result.pandas().xyxy[0]

    # 获取原始图像
    def image(self):
        return self.img

    # 得到检测框的信息xmin, xmax, ymin, ymax, name, confidence
    def value(self):
        xmin = self.results['xmin'].values.astype(int)
        ymin = self.results['ymin'].values.astype(int)
        xmax = self.results['xmax'].values.astype(int)
        ymax = self.results['ymax'].values.astype(int)
        name = self.results['name'].values
        confidence = self.results['confidence'].values
        result = Value(xmin, xmax, ymin, ymax, name, confidence)
        return result
