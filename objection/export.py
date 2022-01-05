import pyzed.sl as sl
import numpy as np
import cv2
from pathlib import Path
import sys
import enum


class AppType(enum.Enum):
    LEFT = 1
    DEPTH = 2


def progress_bar(percent_done, bar_length=50):
    done_length = int(bar_length * percent_done / 100)
    bar = '=' * done_length + '-' * (bar_length - done_length)
    sys.stdout.write('[%s] %f%s\r' % (bar, percent_done, '%'))
    sys.stdout.flush()


def main():
    if not sys.argv or len(sys.argv) != 4:
        sys.stdout.write("Usage: \n\n")
        sys.stdout.write("    ZED_SVO_Export A B C \n\n")
        sys.stdout.write("Please use the following parameters from the command line:\n")
        sys.stdout.write(" A - SVO file path (input) : \"path/to/file.svo\"\n")
        sys.stdout.write(" B - AVI file path (output) or image sequence folder(output) :\n")
        sys.stdout.write("         \"path/to/output/file.avi\" or \"path/to/output/folder\"\n")
        sys.stdout.write(" C - Export mode:  0=Export LEFT AVI.\n")
        sys.stdout.write("                   1=Export DEPTH AVI.\n")
        sys.stdout.write("                   2=Export LEFT image sequence.\n")
        sys.stdout.write("                   3=Export DEPTH image sequence.\n")
        exit()
    svo_input_path = Path(sys.argv[1])
    output_path = Path(sys.argv[2])
    # 是否输出为视频或者图片
    output_as_video = True
    app_type = AppType.LEFT
    if sys.argv[3] == "1":
        app_type = AppType.DEPTH
    # Check if exporting to AVI or SEQUENCE
    if sys.argv[3] == "2" or sys.argv[3] == "3":
        output_as_video = False
    if not output_as_video and not output_path.is_dir():
        sys.stdout.write("Input directory doesn't exist. Check permissions or create it.\n")
        sys.stdout.write(str(output_path))
        sys.stdout.write("\n")
        exit()

    # Create ZED objects
    zed = sl.Camera()
    input_type = sl.InputType()
    # Specify SVO path parameter
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.MILLIMETER
    init.depth_minimum_distance = 1
    init.svo_real_time_mode = False
    init.set_from_svo_file(str(svo_input_path))
    # Open the SVO file specified as a parameter
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        sys.stdout.write(repr(err))
        zed.close()
        exit(1)
    # Get image size
    image_size = zed.get_camera_information().camera_resolution
    width = image_size.width
    height = image_size.height

    # Prepare side by side image container equivalent to CV_8UC4
    svo_image_sbs_rgba = np.zeros((height, width, 4), dtype=np.uint8)

    # Prepare single image containers
    left_image = sl.Mat()
    depth_image = sl.Mat()

    video_writer = None
    if output_as_video:
        # Create video writer with MPEG-4 part 2 codec
        video_writer = cv2.VideoWriter(str(output_path),
                                       cv2.VideoWriter_fourcc('M', '4', 'S', '2'),
                                       max(zed.get_camera_information().camera_fps, 25),
                                       (width, height))

        if not video_writer.isOpened():
            sys.stdout.write("OpenCV video writer cannot be opened. Please check the .avi file path and write "
                             "permissions.\n")
            zed.close()
            exit()
    rt_param = sl.RuntimeParameters()
    rt_param.sensing_mode = sl.SENSING_MODE.STANDARD

    # Start SVO conversion to AVI/SEQUENCE
    sys.stdout.write("Converting SVO... Use Ctrl-C to interrupt conversion.\n")
    nb_frames = zed.get_svo_number_of_frames()
    key = ''
    while key != 113:
        if zed.grab(rt_param) == sl.ERROR_CODE.SUCCESS:
            svo_position = zed.get_svo_position()

            # Retrieve SVO images
            if app_type == AppType.LEFT:
                zed.retrieve_image(left_image, sl.VIEW.RIGHT)
            elif app_type == AppType.DEPTH:
                zed.retrieve_image(depth_image, sl.VIEW.DEPTH)

            if output_as_video:
                # Retrieve SVO images
                if app_type == AppType.LEFT:
                    svo_image_sbs_rgba = left_image.get_data()
                elif app_type == AppType.DEPTH:
                    svo_image_sbs_rgba = depth_image.get_data()
                # Convert SVO image from RGBA to RGB
                ocv_image_sbs_rgb = cv2.cvtColor(svo_image_sbs_rgba, cv2.COLOR_RGBA2RGB)
                # Write the RGB image in the video
                video_writer.write(ocv_image_sbs_rgb)
            else:
                filename = output_path / (("left%s.png" if app_type == AppType.LEFT
                                           else "depth%s.png") % str(svo_position).zfill(6))
                if app_type == AppType.LEFT:
                    ocv_image_sbs_rgb = left_image.get_data()
                    cv2.imwrite(str(filename), ocv_image_sbs_rgb)
                elif app_type == AppType.DEPTH:
                    cv2.imwrite(str(filename), depth_image.get_data())
            # Display progress
            cv2.imshow('out', ocv_image_sbs_rgb)
            progress_bar((svo_position + 1) / nb_frames * 100, 30)

            # Check if we have reached the end of the video
            if svo_position >= (nb_frames - 1):  # End of SVO
                sys.stdout.write("\nSVO end has been reached. Exiting now.\n")
                break
            key = cv2.waitKey(10)
    cv2.destroyAllWindows()
    if output_as_video:
        # Close the video writer
        video_writer.release()
    zed.close()
    return 0


if __name__ == "__main__":
    main()
