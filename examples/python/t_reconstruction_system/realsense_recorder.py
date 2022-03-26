# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2018-2021 www.open3d.org
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
# ----------------------------------------------------------------------------

# examples/python/reconstruction_system/sensors/realsense_recorder.py

# pyrealsense2 is required.
# Please see instructions in https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python
import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
from os import makedirs
from os.path import exists, join, abspath
import shutil
import json
from enum import IntEnum

import sys
sys.path.append(abspath(__file__))
from realsense_helper import get_profiles

try:
    # Python 2 compatible
    input = raw_input
except NameError:
    pass


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


def make_clean_folder(path_folder):
    if not exists(path_folder):
        makedirs(path_folder)
    else:
        user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
        if user_input.lower() == 'y':
            shutil.rmtree(path_folder)
            makedirs(path_folder)
        else:
            exit()


def save_intrinsic_as_json(filename, frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    with open(filename, 'w') as outfile:
        obj = json.dump(
            {
                'width':
                    intrinsics.width,
                'height':
                    intrinsics.height,
                'intrinsic_matrix': [
                    intrinsics.fx, 0, 0, 0, intrinsics.fy, 0, intrinsics.ppx,
                    intrinsics.ppy, 1
                ]
            },
            outfile,
            indent=4)


def sample():
       # Create a pipeline
    pipeline = rs.pipeline()

    #Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    color_profiles, depth_profiles = get_profiles()
    # note: using 640 x 480 depth resolution produces smooth depth boundaries
    #       using rs.format.bgr8 for color image format for OpenCV based image visualization
    print('Using the default profiles: \n  color:{}, depth:{}'.format(
        color_profiles[0], depth_profiles[0]))
    w, h, fps, fmt = depth_profiles[0]
    config.enable_stream(rs.stream.depth, w, h, fmt, fps)
    w, h, fps, fmt = color_profiles[0]
    config.enable_stream(rs.stream.color, w, h, fmt, fps)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()

    # We will not display the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 3  # 3 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Streaming loop
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            yield depth_image, color_image

            # Render images
            #depth_colormap = cv2.applyColorMap(
            #    cv2.convertScaleAbs(depth_image, alpha=0.09), cv2.COLORMAP_JET)
            #images = np.hstack((bg_removed, depth_colormap))
            
    finally:
        pipeline.stop()
    yield None, None
