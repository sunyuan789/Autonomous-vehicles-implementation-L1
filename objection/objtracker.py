from deep_sort.utils.parser import get_config
from deep_sort.deep_sort import DeepSort
import torch
import cv2
from distance import *

cfg = get_config()
cfg.merge_from_file("deep_sort/configs/deep_sort.yaml")
deepsort = DeepSort(cfg.DEEPSORT.REID_CKPT,
                    max_dist=cfg.DEEPSORT.MAX_DIST, min_confidence=cfg.DEEPSORT.MIN_CONFIDENCE,
                    nms_max_overlap=cfg.DEEPSORT.NMS_MAX_OVERLAP, max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                    max_age=cfg.DEEPSORT.MAX_AGE, n_init=cfg.DEEPSORT.N_INIT, nn_budget=cfg.DEEPSORT.NN_BUDGET,
                    use_cuda=True)


def plot_bboxes(image, bboxes, line_thickness=None):
    # Plots one bounding box on image img
    tl = line_thickness or round(
        0.002 * (image.shape[0] + image.shape[1]) / 2) + 1  # line/font thickness
    list_pts = []
    point_radius = 4

    for (x1, y1, x2, y2, pos_id, _) in bboxes:
        # check whether hit line 
        check_point_x = x1
        check_point_y = int(y1 + ((y2 - y1) * 0.6))

        c1, c2 = (x1, y1), (x2, y2)
        cv2.rectangle(image, c1, c2, (0, 255, 0), thickness=tl, lineType=cv2.LINE_AA)
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize('', 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(image, c1, c2, (0, 255, 0), -1, cv2.LINE_AA)  # filled
        cv2.putText(image, 'ID-{}'.format(pos_id), (c1[0], c1[1] - 2), 0, tl / 3,
                    [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

        list_pts.append([check_point_x - point_radius, check_point_y - point_radius])
        list_pts.append([check_point_x - point_radius, check_point_y + point_radius])
        list_pts.append([check_point_x + point_radius, check_point_y + point_radius])
        list_pts.append([check_point_x + point_radius, check_point_y - point_radius])

        ndarray_pts = np.array(list_pts, np.int32)
        cv2.fillPoly(image, [ndarray_pts], color=(0, 0, 255))
        list_pts.clear()
    return image


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


def update(rgb, image_depth):
    image = rgb.image()
    bbox_xywh = []
    confs = []
    bboxes2draw = []
    if len(rgb.results.shape):
        # Adapt detections to deep sort input format
        for i in range(rgb.results.shape[0]):
            obj = [
                int((rgb.value().xmin[i] + rgb.value().xmax[i]) / 2),
                int((rgb.value().ymin[i] + rgb.value().ymax[i]) / 2),
                rgb.value().xmax[i] - rgb.value().xmin[i],
                rgb.value().ymax[i] - rgb.value().ymin[i]
            ]
            bbox_xywh.append(obj)
            confs.append(rgb.value().confidence[i])

        # for x1, y1, x2, y2, _, conf in bboxes:
        #     obj = [
        #         int((x1+x2)/2), int((y1+y2)/2),
        #         x2-x1, y2-y1
        #     ]
        #     bbox_xywh.append(obj)
        #     confs.append(conf)
        xywhs = torch.Tensor(bbox_xywh)
        confss = torch.Tensor(confs)

        # Pass detections to deepsort
        outputs = deepsort.update(xywhs, confss, image)
        for value in list(outputs):
            x1, y1, x2, y2, track_id = value
            gap = abs_distance(image_depth, x1, y1, x2, y2)
            bboxes2draw.append(
                (x1, y1, x2, y2, track_id, round(gap, 2))
            )
    image = plot_bboxes(image, bboxes2draw)
    return image, bboxes2draw
