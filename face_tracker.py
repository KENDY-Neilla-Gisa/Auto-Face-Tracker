import cv2
import numpy as np
import imutils
import serial
import time
from imutils.video import VideoStream

class FaceTracker:
    def __init__(self, serial_port='COM3', baud_rate=9600):
        # Initialize face detector with optimized parameters
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # Initialize video stream with lower resolution for better performance
        self.vs = VideoStream(src=0, resolution=(640, 480), framerate=30).start()
        time.sleep(1.0)  # Reduced warm-up time
        
        # Initialize serial communication with Arduino
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
        time.sleep(2)  # Allow time for Arduino to reset
        
        # Frame dimensions (will be set when processing frames)
        self.frame_width = 0
        self.frame_height = 0
        self.center_x = 0
        self.center_y = 0
        
        # Movement tracking
        self.direction = "CENTER"
        self.vertical_direction = "CENTER"
        self.direction_timer = time.time()
        self.vertical_direction_timer = time.time()
        
        # Visual elements
        self.colors = {
            'green': (0, 255, 0),
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255),
            'white': (255, 255, 255),
            'purple': (255, 0, 255)
        }
        
        # PID controller parameters
        self.prev_error = 0
        self.integral = 0
        self.Kp = 0.5
        self.Ki = 0.01
        self.Kd = 0.1
        
    def process_frame(self):
        # Read frame and check if it's valid
        frame = self.vs.read()
        if frame is None:
            return None
            
        # Resize frame to a smaller size for faster processing
        frame = imutils.resize(frame, width=640)  # Reduced from 800 to 640
        self.frame_height, self.frame_width = frame.shape[:2]
        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2
        
        # Convert to grayscale for face detection (faster processing)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Optimized face detection parameters
        # - scaleFactor=1.05: Slightly faster than 1.1
        # - minNeighbors=3: Fewer checks, faster but slightly less accurate
        # - minSize=(50, 50): Larger minimum size for faster detection
        faces = self.face_cascade.detectMultiScale(
            gray, 
            scaleFactor=1.05, 
            minNeighbors=3, 
            minSize=(50, 50),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        
        # If faces are found, track the largest one
        if len(faces) > 0:
            # Get the largest face (by area)
            (x, y, w, h) = max(faces, key=lambda rect: rect[2] * rect[3])
            
            # Draw rectangle around the face
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Calculate face center
            face_center_x = x + w // 2
            face_center_y = y + h // 2
            
            # Draw center of face and frame center
            cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)
            cv2.circle(frame, (self.center_x, self.center_y), 5, (255, 0, 0), -1)
            
            # Calculate horizontal and vertical errors
            error_x = face_center_x - self.center_x
            error_y = face_center_y - self.center_y
            
            # Update horizontal direction indicator
            if abs(error_x) > 20:  # Only show direction if error is significant
                self.direction = "RIGHT" if error_x > 0 else "LEFT"
                self.direction_timer = time.time()
            elif time.time() - self.direction_timer > 1.0:  # Reset direction after 1 second of no movement
                self.direction = "CENTER"
                
            # Update vertical direction indicator
            if abs(error_y) > 20:  # Only show direction if error is significant
                self.vertical_direction = "DOWN" if error_y > 0 else "UP"
                self.vertical_direction_timer = time.time()
            elif time.time() - self.vertical_direction_timer > 1.0:  # Reset direction after 1 second
                self.vertical_direction = "CENTER"
            
            # Draw center crosshair
            cross_size = 30
            cv2.line(frame, (self.center_x - cross_size, self.center_y), 
                    (self.center_x + cross_size, self.center_y), self.colors['green'], 2)
            cv2.line(frame, (self.center_x, self.center_y - cross_size), 
                    (self.center_x, self.center_y + cross_size), self.colors['green'], 2)
            
            # Draw direction indicators with arrows
            direction_text = f"{self.direction} | {self.vertical_direction}"
            text_size = cv2.getTextSize(direction_text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
            cv2.putText(frame, direction_text, 
                       (self.center_x - text_size[0]//2, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.colors['yellow'], 2, cv2.LINE_AA)
            
            # Draw face center and connecting lines with gradient effect
            cv2.circle(frame, (face_center_x, face_center_y), 8, self.colors['red'], -1)
            cv2.circle(frame, (face_center_x, face_center_y), 12, self.colors['yellow'], 2)
            
            # Draw line from face center to screen center with arrow
            cv2.arrowedLine(frame, (face_center_x, face_center_y), 
                          (self.center_x, self.center_y), 
                          self.colors['purple'], 2, tipLength=0.1)
            
            # Draw horizontal and vertical position indicators
            cv2.rectangle(frame, (10, 10), (210, 110), self.colors['white'], 1)
            cv2.putText(frame, "Position:", (20, 35), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.colors['white'], 1)
            cv2.putText(frame, f"X: {face_center_x - self.center_x:4d}", 
                       (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.colors['yellow'], 1)
            cv2.putText(frame, f"Y: {face_center_y - self.center_y:4d}", 
                       (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.colors['yellow'], 1)
            
            # Calculate errors for both axes
            error_x = face_center_x - self.center_x
            error_y = face_center_y - self.center_y
            
            # PID controller for horizontal movement only
            self.integral = self.integral + error_x
            derivative = error_x - self.prev_error
            output = self.Kp * error_x + self.Ki * self.integral + self.Kd * derivative
            self.prev_error = error_x
            
            # Send command to Arduino with both errors
            self.send_command(error_x, error_y)
        
        # Display the frame (use a smaller window for better performance)
        cv2.imshow("Face Tracker (Press 'q' to quit)", frame)
        
        # Reduce delay in waitKey for more responsive controls
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:  # 'q' or ESC to quit
            return None
            
        return frame
    
    def send_command(self, error_x, error_y):
        """Send command to Arduino based on face position error"""
        try:
            # Deadzone to prevent jitter (adjust as needed)
            deadzone = 30
            
            # Calculate movement amount based on error (proportional control)
            move_amount = min(int(abs(error_x) * 0.5), 100)  # Scale factor to adjust sensitivity
            
            if abs(error_x) > deadzone:
                if error_x > 0:
                    command = f"R{move_amount}"  # Move right
                else:
                    command = f"L{move_amount}"  # Move left
                
                # Send command to Arduino
                self.ser.write((command + '\n').encode())
                
                # For debugging
                print(f"Moving: {command}")
            else:
                # Send stop command if within deadzone
                self.ser.write(b'S\n')
                
        except Exception as e:
            print(f"Error sending command: {e}")
            
        # Small delay to prevent overwhelming the serial port
        time.sleep(0.01)
    
    def cleanup(self):
        """Clean up resources"""
        self.ser.close()
        self.vs.stop()
        cv2.destroyAllWindows()

def main():
    # Initialize face tracker with error handling
    try:
        # Change 'COM3' to your Arduino's serial port
        tracker = FaceTracker(serial_port='COM3')
        
        # Main loop with frame rate control
        prev_frame_time = time.time()
        frame_count = 0
        fps = 0
        
        while True:
            # Process frame and measure FPS
            start_time = time.time()
            frame = tracker.process_frame()
            
            # Calculate and display FPS
            frame_count += 1
            if frame_count % 10 == 0:  # Update FPS every 10 frames
                fps = 10 / (time.time() - prev_frame_time)
                prev_frame_time = time.time()
                print(f"FPS: {fps:.1f}")
                
            if frame is None:
                break
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        tracker.cleanup()

if __name__ == "__main__":
    main()
