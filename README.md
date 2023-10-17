# undistort ``X``

Corrects fisheye distortion based on hypothesis: straight lines are straight

# How to build

Fastest way is to setup a docker image

```
docker build --file container/Dockerfile --tag personal-dev:latest .
```

and run ```spawn.sh``` for container prompt access.

You can then build with:
```
cd SOURCE_DIR
mkdir build
cd build
cmake ..
make
```
</br>

# How to run

```
cd SOURCE_DIR
build/undistortX IMAGE_FILENAME YAML_FILENAME
```

</br>

# API

    undistort(const cv::Mat &input, const DistortionCoefficients& kd)

Undistorts input image with provided coefficients.

    undistort(const cv::Mat &input, const std::vector<std::vector<cv::Point2d>> &pointsDistorted)

Undistorts input image with provided set of points.

    getDistortionCoefficients() -> DistortionCoefficients

Get estimated distortion coefficients, after a call to ```Undistortion::undistort``` Returns coefficient set of 7 params: [ k1, k2, p1, p2, k3, cx, cy ]

    getUndistortedImage() -> cv::Mat

Get undistorted image, after a call to ```Undistortion::undistort```

</br>

Refer to ```sources/main.cpp``` for example API usage.
