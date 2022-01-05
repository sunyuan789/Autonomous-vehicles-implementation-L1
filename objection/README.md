此文件夹包含了使用ZED相机做目标检测和使用ZED相机自带的函数做距离估计，具体实现效果如下

[双目相机行人目标检测与测距_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1mS4y1D7Hp#reply96985235184)

![](https://github.com/sunyuan789/Autonomous-vehicles-implementation-L1/blob/main/objection/demo.png)

## **2022.1.5**

**YOLOV5.py**增加了目标跟踪的代码，借鉴了网络上已有的deepsort模型代码，可实现对行人的目标跟踪

`export.py`文件用于将SVO导出为每一帧原始图片或者avi视频，具体使用方法为：

`python export.py A B C`

A 代表输入的SVO文件名

当C等于0 or 1的时候，B的输入应该为一个目录而不是文件名

当C等于2 or 3的时候，B的输入应该为一个文件名(“建议在文件名前先新建一个目录，此处产生的图片会很多”)

C等于0表示输出为RGB视频，1表示输出为Depth视频

C等于2表示输出每一帧的RGB图片，3表示输出每一帧的Depth图片

Example: 

```shell
python export.py test.svo output/rgb.avi 1
python export.py test.svo output/depth.avi 2
python export.py test.svo output 3
python export.py test.svo output 4
```





