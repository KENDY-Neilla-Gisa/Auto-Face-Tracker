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

In `face_tracker.py`, update the serial port to match your Arduino's port:

```python
tracker = FaceTracker(serial_port='COM3')  # Change 'COM3' to your Arduino's port
```

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

- **No face detected**: Ensure good lighting and that your face is clearly visible
- **Jittery movement**: Try adjusting the PID parameters or increasing the deadband
- **Stepper motor not moving**: Check connections and power supply

## License

This project is open source and available under the MIT License.
"# Auto-Face-Tracker" 
