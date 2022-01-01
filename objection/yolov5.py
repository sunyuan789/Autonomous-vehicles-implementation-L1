import sys

import torch
import pyzed.sl as sl
import cv2
import numpy as np
import warnings

import objtracker
import sympy
import os

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# 加载本地YoloV5模型
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
model = torch.hub.load('yolov5', 'custom', path='yolov5/yolov5s.pt', source='local')


def progress_bar(percent_done, bar_length=50):
    done_length = int(bar_length * percent_done / 100)
    bar = '=' * done_length + '-' * (bar_length - done_length)
    sys.stdout.write('[%s] %f%s\r' % (bar, percent_done, '%'))
    sys.stdout.flush()


# 绘制带背景的文字
def draw_text(img, text,
              font=cv2.FONT_HERSHEY_COMPLEX,
              pos=(0, 0),
              font_scale=1,
              font_thickness=2,
              text_color=(0, 255, 0),
              text_color_bg=(0, 0, 0)
              ):
    x, y = pos
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    cv2.rectangle(img, pos, (x + text_w, y + text_h), text_color_bg, -1)
    cv2.putText(img, text, (x, y + text_h + font_scale - 1), font,
                font_scale, text_color, font_thickness)

    return text_size


# 指定检测框长宽左上角坐标的检测框距离
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
def abs_distance(mat, xmin, ymin, xmax, ymax, distances):
    height = int((ymax - ymin) / 2)
    width = xmax - xmin
    h = 235  # 车辆摄像机距离地面的高度
    d1 = pix_distance(height, width, xmin, ymin, mat)
    d2 = pix_distance(height, width, xmin, ymin + height, mat)
    a = np.square(d1) - np.square(d2)
    x = sympy.Symbol('x')
    out = sympy.solve([x * x - 2 * x * h + a], [x])
    c3 = out[x]
    d = np.sqrt(np.square(distances) - np.square(h) - 0.25 * a + 0.5*h*c3)
    return d


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


def main():
    camera = sl.Camera()
    input_type = sl.InputType()

    # 初始化ZED相机
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.METER
    init.depth_minimum_distance = 1

    # 若有参数输入，则打开文件
    if len(sys.argv) == 2:
        filepath = sys.argv[1]
        print("Using SVO file: {0}".format(filepath))
        init.set_from_svo_file(filepath)

    err = camera.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print(repr(err))
        camera.close()
        exit(1)

    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD

    # 定义相机的分辨率
    image_size = camera.get_camera_information().camera_resolution
    # 定义Mat存储RGBA图像和深度图像
    image_rgb = sl.Mat()
    image_depth_display = sl.Mat()
    image_depth = sl.Mat()

    key = ''
    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    output = cv2.VideoWriter('output.avi', fourcc, 30, (1920, 1080))

    nb_frames = camera.get_svo_number_of_frames()
    # 按下q键就可以退出
    while key != 113:
        err = camera.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            svo_position = camera.get_svo_position()
            # 获取RGBA图像和深度图像
            camera.retrieve_image(image_rgb, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            camera.retrieve_image(image_depth_display, sl.VIEW.DEPTH)
            camera.retrieve_measure(image_depth, sl.MEASURE.DEPTH)
            # 定义获取图像信息类
            rgb = Objection(image_rgb)
            depth = Objection(image_depth_display)
            list_bboxs = []
            # 更新跟踪器
            output_image_frame, list_bboxs = objtracker.update(rgb)

            for i in range(rgb.results.shape[0]):
                # 绘制检测框
                if rgb.value().name[i] == 'person':
                    gap = distance(image_depth,
                                   rgb.value().xmin[i], rgb.value().ymin[i],
                                   rgb.value().xmax[i], rgb.value().ymax[i])
                    cv2.rectangle(rgb.img,
                                  (rgb.value().xmin[i], rgb.value().ymin[i]),
                                  (rgb.value().xmax[i], rgb.value().ymax[i]),
                                  (0, 255, 0), 1, 4)
                    # 打标签类别
                    # + str(round(rgb.value().confidence[i], 2))
                    draw_text(img=rgb.img,
                              text=rgb.value().name[i] + str(round(gap, 2)),
                              pos=(rgb.value().xmin[i], rgb.value().ymin[i]))
            cv2.imshow('rgb', rgb.img)
            cv2.imshow('depth', depth.img)
            output.write(rgb.img)
            key = cv2.waitKey(10)

            progress_bar((svo_position + 1) / nb_frames * 100, 30)
            # 读取到视频文件的最后一帧就退出程序
            if svo_position >= (nb_frames - 1):  # End of SVO
                sys.stdout.write("\nSVO end has been reached. Exiting now.\n")
                break
    output.release()
    cv2.destroyAllWindows()
    camera.close()
    print("Finish")


if __name__ == "__main__":
    main()
