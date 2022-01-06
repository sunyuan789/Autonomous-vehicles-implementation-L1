import sys
import cv2
import objtracker
from distance import *
from objection import *
import os

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

save_txt = True  # 是否将跟踪对象写入txt


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
    output = cv2.VideoWriter('output1.avi',
                             cv2.VideoWriter_fourcc('X', 'V', 'I', 'D'),
                             max(camera.get_camera_information().camera_fps, 30),
                             (1920, 1080))

    nb_frames = camera.get_svo_number_of_frames()
    fw = open('test.txt', 'w+')

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
            output_image_frame, list_bboxs = objtracker.update(rgb, image_depth)

            for i in range(rgb.results.shape[0]):
                # 绘制检测框
                if rgb.value().name[i] == 'person':
                    # gap = distance(image_depth,
                    #                rgb.value().xmin[i], rgb.value().ymin[i],
                    #                rgb.value().xmax[i], rgb.value().ymax[i])
                    gap = abs_distance(image_depth,
                                       rgb.value().xmin[i], rgb.value().ymin[i],
                                       rgb.value().xmax[i], rgb.value().ymax[i])
                    # gap = human_height(image_depth,
                    #                rgb.value().xmin[i], rgb.value().ymin[i],
                    #                rgb.value().xmax[i], rgb.value().ymax[i])
                    # cv2.rectangle(rgb.img,
                    #               (rgb.value().xmin[i], rgb.value().ymin[i]),
                    #               (rgb.value().xmax[i], rgb.value().ymax[i]),
                    #               (0, 255, 0), 1, 4)
                    # 打标签类别
                    draw_text(img=rgb.img,
                              text=rgb.value().name[i] + str(round(gap, 2)),
                              pos=(rgb.value().xmin[i], rgb.value().ymin[i]))
            cv2.imshow('rgb', rgb.img)
            cv2.imshow('depth', depth.img)
            output.write(rgb.img)
            key = cv2.waitKey(10)
            # 将跟踪的ID坐标数据写入txt文档
            if save_txt:
                a = list(list_bboxs)
                for line in a:
                    for x in line:
                        fw.write(str(x))
                        fw.write('\t')
                    fw.write(str(svo_position))
                    fw.write('\n')
            # 显示当前进度
            progress_bar((svo_position + 1) / nb_frames * 100, 30)
            # 读取到视频文件的最后一帧就退出程序
            if svo_position >= (nb_frames - 1):  # End of SVO
                sys.stdout.write("\nSVO end has been reached. Exiting now.\n")
                break
    output.release()
    cv2.destroyAllWindows()
    camera.close()
    fw.close()
    print("Finish")


if __name__ == "__main__":
    main()
