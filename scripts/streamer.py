import rospy
import cv2
import argparse
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv_bridge as br

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true", help="Output node processes information.")
    parser.add_argument("-d", "--display", action="store_true", help="Show current processed image with marker detections.")
    parser.add_argument("-f", "--framerate", type=int, default=20, help="Framerate of images on the output topic. Defaul is 20.")
    config = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    #-> Initialize node
    rospy.init_node("streamer")
    if config.verbose:
        rospy.loginfo("Started node with name 'streamer'")
    
    #-[] Start camera, image2message converter and open visualization window
    camera = cv2.VideoCapture(0)
    bridge = br.CvBridge()
    if config.display:
        cv2.namedWindow("Camera", cv2.WINDOW_AUTOSIZE)
    
    #-[] On node shutdown, close camera and visualization window
    def on_shutdown():
        camera.release()
        if config.display:
            cv2.destroyWindow("Camera")
    rospy.on_shutdown(on_shutdown)

    #-[] Main loop
    #-> Start main publisher
    publisher = rospy.Publisher("color/image_raw", Image, queue_size=1)
    #-> Create a Rate object to fit framerate
    rate = rospy.Rate(config.framerate)
    #-? seq is for the message header
    seq = 0
    while not rospy.is_shutdown():
        #-[] Test if camera is opened
        if not camera.isOpened():
            if config.verbose:
                rospy.logwarn("Camera not opened, skipping loop pass.")
            continue

        #-[] Try to retrieve an image
        if config.verbose:
            rospy.loginfo("Trying to retrieve an image.")
        ret, image = camera.read()
        if not ret:
            if config.verbose:
                rospy.logwarn("No image retrieved, skipping loop pass.")
            continue
        if config.verbose:
            rospy.loginfo("Image retrieved.")

        if config.display:
            if config.verbose:
                rospy.loginfo("Displaying image.")
            cv2.imshow("Camera", image)
            cv2.waitKey(1)

        #-[] Create image message
        #-> Convert image into suitable publishing format
        image_msg = bridge.cv2_to_imgmsg(image)
        #-> Fill image's header
        header = Header()
        header.seq = seq
        seq += 1
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_color_optical_frame"
        image_msg.header = header

        #-> Send image message
        if config.verbose:
            rospy.loginfo("Sending image message.")
        publisher.publish(image_msg)

        #-> Finish loop pass by waiting time needed to match framerate
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass