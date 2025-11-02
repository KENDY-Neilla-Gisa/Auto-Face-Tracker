import cv2

def test_camera():
    # Try different camera indices (0 to 4)
    for i in range(5):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"\nCamera {i} is working!")
            ret, frame = cap.read()
            if ret:
                print(f"  Resolution: {frame.shape[1]}x{frame.shape[0]}")
                # Show the camera feed
                while True:
                    ret, frame = cap.read()
                    if not ret:
                        break
                    cv2.imshow(f"Camera {i} - Press 'q' to exit", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
            cap.release()
            cv2.destroyAllWindows()
        else:
            print(f"No camera found at index {i}")

if __name__ == "__main__":
    print("Testing available cameras...")
    test_camera()
