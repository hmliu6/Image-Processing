# Circle Location with Image

## Requirement

For python codes, this project is based on python 2.7 and Mac OS or Ubuntu 16.04.

For C++ codes, this project can be compiled on Windows 10 or Mac OS or Ubuntu 16.04 or above.

* OpenCV 3.3.1

* Python 2.7

## Installation

### Mac OS environments

1. Install `homebrew` which has similar functions as `apt-get install` in ubuntu

2. RUN `brew install opencv` in your local machine

## Stage

### Finished

* Recognize shorter fence which have clear depth returned values

* Algorithms calculate dynamically and not relied on fixed parameters

* Ignore objects near fences

* Process with given images or videos

* Locate throwing balls

### Bug

* Few errors when not stable moving in live video (Will be fixed in kinect detection project, use array to store sequences and check)

* Recognize longer fence

### Not finished

* Goal Detection

## Usage

### File: ${FILENAME}.cpp

#### Mac User

```
bash OpenCV-compile.sh ${FILENAME}
```

* Probably works on ubuntu also, but not tested

### File: filter.py

Prototype for processing image and re-draw the fence. For more options,
```
python filter.py --help
```

### File: filtering.cpp

Re-draw and calculate fence based on input images with more efficient way
```
bash OpenCV-compile.sh filtering
```

### File: video-opening.cpp

Re-draw and calculate fence based on input video (Discontinuous, only for testing)
```
bash OpenCV-compile.sh video-opening
```

### File: shield.cpp

Current idea: detect throwing ball with removing background and get contours
```
bash OpenCV-compile.sh shield
```