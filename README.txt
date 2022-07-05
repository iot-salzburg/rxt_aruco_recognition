Things To remember:

1. device name for USB camera is randomly changing need to fix that
2. Calbration of the camera is required just use take_snaps.py file and hold the chessboard and press "space" to take snaps and it will save in "snapshots" folder. the python and chessboard can be found in callibration folder.
3. after taking more than 25 snaps now run the cam_calib.py like "python cameracalib.py <folder> <image type> <num rows (9)> <num cols (6)> <cell dimension (25) in mm". I used like "python cameracalib.py /home/panda/ros_workspace/src/arti_docking/scripts/calibration jpg 9 6 26"
4. After 2 text files will be created which will be located in "snapshots" folder copy those files and drop those in "arti_docking/scripts".
5. Finally run "aruco_detection.py"


aruco_detection.py:
which  will print 4 important values

id        - ID of the respective aruco marker
x_center  - center x_position of the detected Aruco marker
y_center  - center x_position of the detected Aruco marker
theta/yaw - orientation of the Aruco marker

to listen to the topic use:

rostopic echo /aruco_data
