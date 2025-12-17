##Program that sends Camera feed from the Pi to the PC

import socket
import struct
import cv2

HOST = "0.0.0.0"   # Listen on all interfaces
PORT = 8000        # You can change this, but use the same in the client

def main():
    # Open the USB camera (usually index 0)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Optionally set resolution (uncomment & adjust if you want)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Create TCP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)

    print(f"Server listening on {HOST}:{PORT} ...")
    conn, addr = server_socket.accept()
    print(f"Client connected from {addr}")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to read frame from camera.")
                break

            # Encode frame as JPEG
            ret, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ret:
                print("Error: Failed to encode frame.")
                break

            data = jpeg.tobytes()
            # Prefix each frame with a 4-byte length (big endian)
            size = struct.pack(">I", len(data))

            # Send size and then the frame
            conn.sendall(size + data)

    except (BrokenPipeError, ConnectionResetError):
        print("Client disconnected.")
    finally:
        cap.release()
        conn.close()
        server_socket.close()
        print("Server shutdown.")

if __name__ == "__main__":
    main()
