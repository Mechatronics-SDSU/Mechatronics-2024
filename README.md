# Dependancies
* `pip install torch`
* `pip install opencv-python`
* `pip install ultralytics`
* `pip install yolov5`

# install Instructions for zed sdk

[dependancy installation](https://www.stereolabs.com/docs/app-development/python/install/)

[zed sdk installation](https://www.stereolabs.com/docs/installation/linux/)

[Additional Info on their github](https://github.com/stereolabs/zed-sdk)


# Running Instructions

`python3 run_video_over_socket`

### arguments(all are optional)

`-host_ip`      which host do you want to send images to(ipv4)

`-port`         which port to send images over

`-show_boxes`   boolean, do you want to bounding boxes on the image

`-model_name`   which model(expects.pt) do you want to load

### example command
`python3 run_video_over_socket -host_ip 127.0.01 -port 8089 -show_boxes False -model_name best.pt`


# Possible Errors
`cv2.error: OpenCV(4.8.0) /io/opencv/modules/highgui/src/window.cpp:1255: error: 
(-2:Unspecified error) 
The function is not implemented. Rebuild the library with Windows, GTK+ 2.x or Cocoa support. 
If you are on Ubuntu or Debian, install libgtk2.0-dev and pkg-config, 
then re-run cmake or configure script in function 'cvNamedWindow'`

Found Solution
```
pip uninstall opencv-python()
pip3 install opencv-python
