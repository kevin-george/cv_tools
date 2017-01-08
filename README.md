# List of tools

## capture_frames_from_video.py
This is a simple module that reads frames from a video

#### Usage:
`python capture_frames_from_video.py <video>`
#### Dependencies:
opencv 2.4/3.0

<br/><br/>
## calibrate_camera.py
This module takes a series of images of an asymmetric chessboard as input and returns the camera intrinsics  
#### Usage:   
`python calibrate_camera.py <directory_of_images> <image_extension>`
#### Dependencies:
opencv 2.4/3.0
#### Details:
* [An excellent analysis of Camera Intrinsics](https://ksimek.github.io/2013/08/13/intrinsic/)

<br/><br/>
## calculate_fmatrix.py
This module takes a pair of images as input and returns the Fundamental Matrix
#### Usage:
`python calculate_fmatrix.py <image 1> <image2>`  
`python calculate_fmatrix.py --decompose <intrinsics_file> <image 1> <image2>`
#### Dependencies:
opencv,
#### Details:
* [Epipolar Geometry and the Fundamental Matrix](http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook1/HZepipolar.pdf)     
* [The Fundamental Matrix Song](http://danielwedge.com/fmatrix/)

<br/><br/>
## calculate_homography.py
This module takes a pair of images of a planar surface and returns the Homography
#### Usage:
`python calculate_homography.py <image 1> <image2>`  
`python calculate_homography.py --decompose <intrinsics_file> <image 1> <image 2>`
#### Dependencies:
opencv,
#### Details:
* [Homography](http://people.scs.carleton.ca/~c_shu/Courses/comp4900d/notes/homography.pdf)
* Since opencv hasn't exposed decomposeHomographyMat to the python bindings yet, I wrote my own implementation following
the paper [Malis, E. and Vargas, M. Deeper understanding of the homography decomposition for vision-based control, Research Report 6303, INRIA (2007)](https://hal.archives-ouvertes.fr/file/index/docid/174739/filename/RR-6303.pdf)