# Auto Face Tracker

An automated face tracking system that uses computer vision to detect and track faces in real-time, controlling a stepper motor to keep the camera centered on the detected face.

## Features

- Real-time face detection using OpenCV
- Smooth camera movement with PID control
- Serial communication between Python and Arduino
- Adjustable tracking parameters
- Works with standard webcams and USB cameras

## Hardware Requirements

- Webcam or USB camera
- Arduino board (Uno, Nano, or similar)
- 28BYJ-48 stepper motor with ULN2003 driver
- Jumper wires
- Power supply for the stepper motor (5V-12V)

## Setup Instructions

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Arduino Setup

1. Connect the stepper motor to the Arduino:
   - IN1 -> Pin 8
   - IN2 -> Pin 9
   - IN3 -> Pin 10
   - IN4 -> Pin 11
   - Connect power supply to the ULN2003 driver

2. Upload the `face_tracker_arduino.ino` sketch to your Arduino board using the Arduino IDE.

### 3. Configure Serial Port

In `face_tracker.py`, update the serial port and baud rate to match your Arduino's settings:

```python
tracker = FaceTracker(serial_port='COM3', baud_rate=115200)  # Change 'COM3' to your Arduino's port
```

> **Note:** The default baud rate in the Arduino code is 115200. Make sure this matches the baud rate in your Python code.

### 4. Run the Face Tracker

```bash
python face_tracker.py
```

## Usage

- Position yourself in front of the camera
- The system will detect your face and track it
- Press 'q' to quit the application

## Adjusting Tracking Parameters

You can adjust the PID controller parameters in the `FaceTracker` class for smoother or more responsive tracking:

```python
# PID controller parameters
self.Kp = 0.5  # Proportional gain
self.Ki = 0.01  # Integral gain
self.Kd = 0.1   # Derivative gain
```

## Troubleshooting

### Camera Issues
- **No camera feed**:
  - Check if another application is using the camera
  - Verify the camera is properly connected and recognized by your system
  - Try running `camera_test.py` to verify camera functionality

### Serial Communication
- **Serial port errors**:
  - Verify the correct COM port is specified in `face_tracker.py`
  - Check if the Arduino is properly connected to your computer
  - Ensure no other program is using the serial port (like the Arduino IDE Serial Monitor)
  - Verify the baud rate matches in both Arduino code (9600) and Python code

### Motor Issues
- **Motor not moving**:
  - Check all motor connections (IN1-IN4 to Arduino pins 8-11)
  - Verify the power supply is connected to the ULN2003 driver
  - Ensure the power supply provides enough current (5V-12V, at least 200mA)
  - Listen for any unusual sounds from the motor (may indicate wiring issues)

### Performance Issues
- **Laggy tracking**:
  - Reduce the camera resolution in `face_tracker.py`
  - Close other resource-intensive applications
  - Ensure good lighting conditions for better face detection

### Common Errors
- **`ImportError: No module named 'cv2'**:
  - Run `pip install -r requirements.txt` to install all dependencies
- **`SerialException: could not open port 'COMx'**:
  - Check if the Arduino is connected and the port is correct
  - Try unplugging and reconnecting the Arduino
  - Restart your computer if the issue persists and power supply

## License

This project is open source and available under the MIT License.
"# Auto-Face-Tracker"
