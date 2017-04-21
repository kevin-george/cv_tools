# Dependencies
[Installing opencv2.x with python2.x bindings](https://gist.github.com/kevin-george/955e1d20d2305dbe3b1e8e6675230268)

## capture_frames_from_video.py
This is a simple module that reads frames from a video

## calibrate_camera.py
This module takes a series of images of an asymmetric chessboard as input and returns the camera intrinsics  
### More Info:
* [An excellent analysis of Camera Intrinsics](https://ksimek.github.io/2013/08/13/intrinsic/)

## calculate_fmatrix.py
This module takes a pair of images as input and returns the Fundamental Matrix. This is an algebraic representation of 
the epipolar geometry that is assumed to be true between the two images. 
### More Info:
* [Epipolar Geometry and the Fundamental Matrix](http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook1/HZepipolar.pdf)     
* [The Fundamental Matrix Song](http://danielwedge.com/fmatrix/)

## calculate_homography.py
This module takes a pair of images of a planar surface and returns the Homography. Since opencv hasn't exposed decomposeHomographyMat to the python bindings yet, I wrote my own implementation following the paper [Malis, E. and Vargas, M. Deeper understanding of the homography decomposition for vision-based control, Research Report 6303, INRIA (2007)](https://hal.archives-ouvertes.fr/file/index/docid/174739/filename/RR-6303.pdf)
### More Info:
* [Homography](http://people.scs.carleton.ca/~c_shu/Courses/comp4900d/notes/homography.pdf)
