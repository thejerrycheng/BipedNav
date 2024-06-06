import pyrealsense2 as rs
import numpy as np
import cv2
import copy
import math
import argparse
import os

class ARC:
    def __init__(self, bag_file):
        self.pipeline = rs.pipeline()
        self.bag_file = bag_file

        config = rs.config()
        config.enable_device_from_file(bag_file, False)
        config.enable_all_streams()

        profile = self.pipeline.start(config)
        device = profile.get_device()
        playback = device.as_playback()
        playback.set_real_time(False)

        self.create_output_dirs()

    def create_output_dirs(self):
        base_name = os.path.splitext(os.path.basename(self.bag_file))[0]
        self.rgb_dir = base_name + "_rgb"
        self.depth_dir = base_name + "_depth"

        if not os.path.exists(self.rgb_dir):
            os.makedirs(self.rgb_dir)
        if not os.path.exists(self.depth_dir):
            os.makedirs(self.depth_dir)

    def video(self):
        align_to = rs.stream.depth
        align = rs.align(align_to)
        frame_count = 0
        for i in range(10):
            self.pipeline.wait_for_frames()
        while True:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            self.depth_frame = depth_frame
            self.color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

            color_cvt = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            # Save the aligned images
            self.save_images(color_cvt, depth_image, frame_count)

            frame_count += 1

            # Display aligned images (optional)
            # self.show(color_cvt, depth_image)

            if frame_count >= 100:  # Limiting to 100 frames for this example
                break

    def save_images(self, color_img, depth_img, frame_count):
        rgb_filename = os.path.join(self.rgb_dir, f"frame_{frame_count:04d}.png")
        depth_filename = os.path.join(self.depth_dir, f"frame_{frame_count:04d}.png")

        cv2.imwrite(rgb_filename, color_img)
        cv2.imwrite(depth_filename, depth_img)

    def show(self, color_img, depth_img):
        self.img_origin = color_img
        self.img_copy = copy.copy(self.img_origin)
        cv2.namedWindow("Color Stream", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("Aligned Color Image", color_img)
        cv2.imshow("Aligned Depth Image", depth_img)

        cv2.setMouseCallback("Color Stream", self.draw)
        while True:
            cv2.imshow("Color Stream", self.img_copy)
            key = cv2.waitKey(10)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

    def draw(self, event, x, y, flags, params):
        img = copy.copy(self.img_copy)
        if event == 1:
            self.ix = x
            self.iy = y
        elif event == 4:
            img = self.img_copy
            self.img_work(img, x, y)
        elif event == 2:
            self.img_copy = copy.copy(self.img_origin)
        elif flags == 1:
            self.img_work(img, x, y)
            cv2.imshow("Color Stream", img)

    def img_work(self, img, x, y):
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        fontColor = (0, 0, 0)
        lineType = 2

        ans = self.calculate_distance(x, y)
        cv2.line(img, pt1=(self.ix, self.iy), pt2=(x, y), color=(255, 255, 255), thickness=3)
        cv2.rectangle(img, (self.ix, self.iy), (self.ix + 80, self.iy - 20), (255, 255, 255), -1)
        cv2.putText(img, '{0:.5}'.format(ans), (self.ix, self.iy), font, fontScale, fontColor,
                    lineType)

    def calculate_distance(self, x, y):
        color_intrin = self.color_intrin
        ix, iy = self.ix, self.iy
        udist = self.depth_frame.get_distance(ix, iy)
        vdist = self.depth_frame.get_distance(x, y)

        point1 = rs.rs2_deproject_pixel_to_point(color_intrin, [ix, iy], udist)
        point2 = rs.rs2_deproject_pixel_to_point(color_intrin, [x, y], vdist)

        dist = math.sqrt(
            math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1], 2) + math.pow(
                point1[2] - point2[2], 2))
        return dist

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Align RGB and Depth images from a RealSense bag file.")
    parser.add_argument('bag_file', type=str, help='Path to the bag file')
    args = parser.parse_args()

    ARC(args.bag_file).video()
