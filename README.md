# SaraKIT Face Analysis: Face Tracking for Raspberry Pi 4 CM4

SaraKIT is an easy-to-use face analysis solution for Raspberry Pi 4 CM4, powered by state-of-the-art algorithms based on MediaPipe from Google. It provides robust functionality for face detection, face landmark detection, and face mesh processing, specifically optimized for the Raspberry Pi 64-bit platform.

**Discover the Power of SaraKIT's Pan-Tilt Camera with Advanced Face Detection and Tracking!** Our pan tilt camera system, based on the SaraKIT platform, offers excellent face detection capabilities and the ability to track facial movements. By utilizing quiet, precise, and fast BLDC Gimbal motors, the camera can smoothly move in response to facial motions, ensuring precise and accurate tracking. This innovative solution represents another way to leverage SaraKIT in projects related to artificial intelligence and home automation. It enables integration with AI systems and home automation, opening up new possibilities for monitoring, facial recognition, and real-time interaction.

## Getting Started

To utilize SaraKIT for face analysis, follow these steps:

1. Install OpenCV & Mediapipe (https://github.com/SaraEye/Install-OpenCV-Bazel-MediaPipe-Raspberry-Pi-64-bits)
2. Compile the code by running the command `make`.
3. Execute the program using `./FaceTracking`.
4. The program captures frames from the camera, processes them, and sends the output.
5. Preview the operation in your web browser by accessing the Raspberry Pi's IP address followed by port 7777 (e.g., http://raspberrypi:7777 or http://192.168.1.16:7777).<br>
if you have the Linux Desktop version and want to display the image from the camera in a window, change this line:<br>
init_viewer(ViewMode::Camera0,ViewMode::Processed, 1, true, false);
6. The browser preview displays one or two images side by side, where the content of each image can be customized. By default, the left image shows the camera preview, while the right image displays the detected face along with face landmarks. Refer to the video below for a similar visualization.

Both the standard Raspberry Pi MMAL functions and OpenCV functions can be used to capture frames from the camera.

For instructions on how to install MediaPipe on Raspberry Pi, please refer to our separate repository dedicated to this topic.

## See more

### SaraKIT - Color Tracking:

![SaraKIT_ColorTracking](https://github.com/SaraEye/SaraKIT-Face-Tracking-MediaPipe-Raspberry-Pi-64bit/assets/35704910/7120ee8a-7612-4d82-8a3c-8cf38c451009)

[Video Link](https://saraai.com/_SaraKIT/video/SaraKIT_ColorTracking.mp4)

### SaraKIT - Face Detection, Face Mesh:

![SaraKIT_Face](https://github.com/SaraEye/SaraKIT-Face-Tracking-MediaPipe-Raspberry-Pi-64bit/assets/35704910/5aebf67d-e821-4c28-b48f-2b3540df5b75)

[Video Link](https://saraai.com/_SaraKIT/video/SaraKIT_Face.mp4)

### SaraKIT - Object Detection:

![SaraKIT_Object_Detection](https://github.com/SaraEye/SaraKIT-Face-Tracking-MediaPipe-Raspberry-Pi-64bit/assets/35704910/e76bcf47-3d1c-4179-a330-c69601f94a8a)

[Video Link](https://saraai.com/_SaraKIT/video/SaraKIT_Object_Detection.mp4)

### SaraKIT - Field Oriented Control (FOC):

![SaraKIT_field_oriented_control_FOC](https://github.com/SaraEye/SaraKIT-Face-Tracking-MediaPipe-Raspberry-Pi-64bit/assets/35704910/e93a09ee-23c4-44a3-a6e2-f6a2e64c351b)

[Video Link](https://saraai.com/_SaraKIT/video/SaraKIT_field_oriented_control_FOC.mp4)

### SaraKIT - BLDC Gimbal Motors (FOC) vs Servo vs Stepper motor

[YouTube Link](https://youtu.be/Nwvnoo5efzE)

## SaraAI LLC

Website: [https://SaraAI.com](https://SaraAI.com)<br>
Project Page: [https://SaraKIT.SaraAI.com](https://SaraKIT.SaraAI.com)
