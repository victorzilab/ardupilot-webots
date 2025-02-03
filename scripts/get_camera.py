
import cv2
import socket
import struct
import numpy as np

# conectando na stream de video.  
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5599))

header_size = struct.calcsize("=HH")
while True:
    header = s.recv(header_size)
    if len(header) != header_size:
        print("Header size mismatch")
        break

    width, height = struct.unpack("=HH", header)

    bytes_to_read = width * height * 3  # 3 bytes per pixel for RGB
    img = bytes()
    while len(img) < bytes_to_read:
        img += s.recv(min(bytes_to_read - len(img), 4096))

    img = np.frombuffer(img, np.uint8).reshape((height, width, 3))  # 3 channels for RGB

    # Bruxarias digitais podem ser feitas usando o objeto "img" a partir daqui
    
    cv2.imshow("RGB Image", img)
    
    if cv2.waitKey(1) == ord("q"):
        break

s.close()
