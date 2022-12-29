#! /usr/bin/python3

import argparse
import os
import shutil
import sys
import tarfile

import cv2
import numpy as np
import rospy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true", help="Output node processes information.")
    parser.add_argument("-t", "--time", type=int, default=10, help="Duration in seconds of the recording. Default is 10.")
    parser.add_argument("-f", "--fps", type=int, default=20, help="Frames per seconds. Default is 20, max is 20.")
    parser.add_argument("-o", "--output", type=str, default="/tmp/foo.tar.gz", help="Name of the images tarball. Default is '/tmp/foo.tar.gz'")
    config = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("recorder")
    if config.verbose:
        rospy.loginfo("Node started with name 'recorder'")

    try:
        #-> Calculate nb of frames to retrieve and inter-frame delay
        fps = config.fps % 21
        number = config.time * fps
        delay = 1000 / fps
        timer = rospy.Rate(fps)

        #-> Open camera
        cam = cv2.VideoCapture(0)

        #-[] Record images
        images = []
        idx = 0
        while idx < number and cam.isOpened() and not rospy.is_shutdown():
            #-> Retrieve an image
            ret, image = cam.read()
            if not ret:
                continue
            #-> Place it in buffer
            images.append(image)
            #-> Finish the loop pass
            idx += 1
            timer.sleep()

        #-> Close camera
        cam.release()

        #-[] Save images into a tarball
        dir = "/tmp/recorder_images/"
        if os.path.exists(dir):
            shutil.rmtree(dir)
        os.mkdir(dir)
        with tarfile.open(config.output, "w:gz") as tar:
            for idx, image in enumerate(images):
                name = f"image_{idx}.npy"
                np.save(os.path.join(dir, name), image)
                tar.add(os.path.join(dir, name))
        shutil.rmtree(dir)

    except rospy.ROSInterruptException:
        return

if __name__ == "__main__":
    main()
