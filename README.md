# SaraKIT Face Analysis: Face Tracking for Raspberry Pi 4 CM4

SaraKIT is an easy-to-use face analysis solution for Raspberry Pi 4 CM4, powered by state-of-the-art algorithms based on MediaPipe from Google. It provides robust functionality for face detection, face landmark detection, and face mesh processing, specifically optimized for the Raspberry Pi 64-bit platform.

"Discover the Power of SaraKIT's Pan-Tilt Camera with Advanced Face Detection and Tracking!" Our pan tilt camera system, based on the SaraKIT platform, offers excellent face detection capabilities and the ability to track facial movements.
By utilizing quiet, precise, and fast BLDC Gimbal motors, the camera can smoothly move in response to facial motions, ensuring precise and accurate tracking. This innovative solution represents another way to leverage SaraKIT in projects related to artificial intelligence and home automation. It enables integration with AI systems and home automation, opening up new possibilities for monitoring, facial recognition, and real-time interaction.


To utilize SaraKIT for face analysis, follow these steps:

1. Install OpenCV & Mediapipe (https://github.com/SaraEye/Install-OpenCV-Bazel-MediaPipe-Raspberry-Pi-64-bits)
2. Compile the code by running the command `make`.
3. Execute the program using `./FaceTracking`.
4. The program captures frames from the camera, processes them, and sends the output.
5. Preview the operation in your web browser by accessing the Raspberry Pi's IP address followed by port 7777 (e.g., http://192.168.1.16:7777).
6. The browser preview displays one or two images side by side, where the content of each image can be customized. By default, the left image shows the camera preview, while the right image displays the detected face along with face landmarks. Refer to the video below for a similar visualization.

Both the standard Raspberry Pi MMAL functions and OpenCV functions can be used to capture frames from the camera.

For instructions on how to install MediaPipe on Raspberry Pi, please refer to our separate repository dedicated to this topic.
