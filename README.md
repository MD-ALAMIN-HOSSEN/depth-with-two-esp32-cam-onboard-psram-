
2.1 Conceptual Framework
The proposed stereo vision system follows a dual-ESP32-CAM architecture, where one module acts as an Access Point (AP) and the other as a Client. The framework includes:
	Image Capture: Grayscale images (160×120 resolution) are acquired simultaneously.
	Preprocessing: Gaussian blur (3×3 kernel) reduces noise.
	Disparity Estimation: Sum of Absolute Differences (SAD) block matching (7×7 blocks) computes horizontal disparities.
	Depth Calculation: Triangulation (Depth = (Focal Length × Baseline) / Disparity) converts disparities to real-world distances.
	3D coordinates : Pixel positions mapped into 3D world coordinates (X, Y, Z) for visualization.
	Data Transmission: HTTP endpoints facilitate wireless communication between modules.
Justification: This framework balances accuracy and computational efficiency, leveraging ESP32’s PSRAM for image buffering and HTTP for low-latency data exchange.








Data :
Block Size	Scenario	Depth Calc Time (milliseconds)	Response Time (milliseconds)	Distance (centimeters)
7×7	Empty	2006	4459	255 (capped)
7×7	Object	1681	3945	35
9×9	Empty	2491	5378	255 (capped)
9×9	Object	2843	6426	35
 
