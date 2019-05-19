<p align = "center">
<img src="https://github.com/FinleyPan/Instant-3D-Scan-Capturer/blob/master/result/view1.png" alt="view1" height="300"> <img src="https://github.com/FinleyPan/Instant-3D-Scan-Capturer/blob/master/result/view2.png" alt="view2" height="300">
 </p>

# Dependencies
- CMake
- [OpenCV](https://github.com/opencv/opencv)(version >= 3)
- [libfreenect2](https://github.com/OpenKinect/libfreenect2)
- [PCL](https://github.com/PointCloudLibrary/pcl)

# How to build
#### on Linux(has been tested on Ubuntu 16.04):
```
$ git clone https://github.com/FinleyPan/Instant-3D-Scan-Capturer Instant-3D-Scan-Capturer 
$ cd ../Instant-3D-Scan-Capturer
$ mkdir build 
$ cd build
$ cmake ..
$ make
```
# Usage
- running executable:
```
$ cd ../Instant-3D-Scan-Capturer/build
$ ./3DScanCap
```
- Press `s` for saving instant 3D Scan, `Esc` or `Ctrl+c` for quit.
- type `pcl_viewer map.pcd` to check generated 3D scan.
