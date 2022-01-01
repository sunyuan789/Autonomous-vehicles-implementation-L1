########################################################################
#
# Copyright (c) 2021, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import sys
import pyzed.sl as sl
from signal import signal, SIGINT

cam = sl.Camera()

def handler(signal_received, frame):
    cam.disable_recording()
    cam.close()
    sys.exit(0)

signal(SIGINT, handler)

def main():
    # 输入参数，需要输入一个用于保存的文件名，
    # python svo_recording.py test.svo
    if not sys.argv or len(sys.argv) != 2:
        print("Only the path of the output SVO file should be passed as argument.")
        exit(1)
    # 设置相机参数,resolution是分辨率,depth_mode是设置深度显示模式，NONE指的是不记录深度信息
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.depth_mode = sl.DEPTH_MODE.NONE
    # 打开相机并检查是否打开，若打开失败则返回错误码
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit(1)
    # 存储输出路径和名称
    path_output = sys.argv[1]
    # 数据保存路径和压缩格式
    recording_param = sl.RecordingParameters(path_output, sl.SVO_COMPRESSION_MODE.H264)
    # 创建SVO文件并依据上一步信息开始录制
    err = cam.enable_recording(recording_param)
    # 检查错误码信息
    if err != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit(1)
    # 定义一个图像显示类，包含运行时的参数结构
    runtime = sl.RuntimeParameters()
    print("SVO is Recording, use Ctrl-C to stop.")
    frames_recorded = 0

    while True:
        # 相机依据runtime提供的参数结构，开始捕获当前相机帧的图像，若没有出错将保存并记录
        if cam.grab(runtime) == sl.ERROR_CODE.SUCCESS :
            frames_recorded += 1
            print("Frame count: " + str(frames_recorded), end="\r")

if __name__ == "__main__":
    main()