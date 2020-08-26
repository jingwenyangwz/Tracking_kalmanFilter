# Tracking_kalmanFilter
Develop both constant velocity model and constant acceleration model for object tracking based on the Kalman filter framework using the OpenCV C++ library; Analyze the strengths and weaknesses of the different modules that compose the pipeline of the algorithms.
- task1:
  - OpenCV MOG is used for foreground detection (parameters for MOG: learning_rate=0.001, varThreshold=16 and history=50)
  - Apply morphological opening to filter noise from the foreground mask (parameters for Opening: size=3x3, type=MORPH_RECT)
  - Apply blob detection to the filtered mask and get as measurement (for Kalman) the center of the biggest blob found in the mask (suggested parameters for blob extraction: min width=10, min height=10) Mandatory requirements for Kalman filtering:
  - Use the OpenCV KalmanFilter class
    - Implement two models: constant velocity model and constant acceleration model


- task2:
analyze the effect of Kalman components for toy video sequences while keeping fixed measurement extraction parameters


- task3:
analyze and tune the Kalman‐based tracker  in real video sequences from [changedetection.net](http://changedetection.net/)


