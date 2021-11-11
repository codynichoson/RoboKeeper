import pyrealsense2 as rs
import numpy as np 
import cv2

# def callback(x):
#     global H_low,H_high,S_low,S_high,V_low,V_high
    #assign trackbar position value to H,S,V High and low variable

def callback(x):
    pass


#HSV VALUES FOR THE STUFF

#HSV FOR RED
#low h:
#low s:
#low v:
#high h:
#high s:
#high v:


cfg = rs.pipeline()  #good
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_record_to_file("camera_video")
profile = cfg.start(config) #good

p= profile.get_stream(rs.stream.color)
intrinsics = p.as_video_stream_profile().get_intrinsics()
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

align_to = rs.stream.color
align = rs.align(align_to)
work = True

cv2.namedWindow("HSV Value")
cv2.createTrackbar("H MIN", "HSV Value", 0, 179, callback)
cv2.createTrackbar("S MIN", "HSV Value", 0, 255, callback)
cv2.createTrackbar("V MIN", "HSV Value", 0, 255, callback)
cv2.createTrackbar("H MAX", "HSV Value", 179, 179, callback)
cv2.createTrackbar("S MAX", "HSV Value", 255, 255, callback)
cv2.createTrackbar("V MAX", "HSV Value", 255, 255, callback)
try:
    while work:
        frames = cfg.wait_for_frames()

        aligned_frames = align.process(frames)


        

        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        #detect purple
        hsv = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV)
        h_min = cv2.getTrackbarPos("H MIN", "HSV Value")
        s_min = cv2.getTrackbarPos("S MIN", "HSV Value")
        v_min = cv2.getTrackbarPos("V MIN", "HSV Value")
        h_max = cv2.getTrackbarPos("H MAX", "HSV Value")
        s_max = cv2.getTrackbarPos("S MAX", "HSV Value")
        v_max = cv2.getTrackbarPos("V MAX", "HSV Value")
        lower_purple = np.array([h_min, s_min, v_min])
        upper_purple = np.array([h_max, s_max, v_max])
        # lower_purple = np.array([[110, 120, 60]])
        # upper_purple = np.array([130, 255, 190])
        # lower_purple = np.array([[150, 0, 60]])
        # upper_purple = np.array([200, 10, 100])

        mask = cv2.inRange(hsv, lower_purple, upper_purple)
        blur = cv2.GaussianBlur(mask,(3,3),0)
        res = cv2.bitwise_and(bg_removed,bg_removed, mask=mask)
        
        num_cont = -1

        contours, hierarchy = cv2.findContours(blur,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(bg_removed, contours, num_cont, (0,0,255), 3)
        
       
        cv2.imshow('mask',mask)
        cv2.imshow('mask_blur',blur)
        cv2.imshow('res',res)


        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', images)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    cfg.stop()
cv2.destroyAllWindows()