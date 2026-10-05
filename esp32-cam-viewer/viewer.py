"""Shows the ESP32-S3 camera feed streamed over USB. Press q or Esc to quit."""
import struct
import sys
import time

import cv2
import numpy as np
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem3101"
MAGIC = b"FRAME"
MAX_FRAME = 500_000


def frames(ser):
    buf = bytearray()
    while True:
        buf += ser.read(ser.in_waiting or 1)
        while True:
            i = buf.find(MAGIC)
            if i < 0:
                # Print any text the board sends (e.g. init errors), keep a tail for split magic
                text = bytes(buf[:-len(MAGIC)])
                if b"ERROR" in text:
                    print(text.decode(errors="replace").strip())
                del buf[:-len(MAGIC)]
                break
            if len(buf) < i + 9:
                break
            (length,) = struct.unpack_from("<I", buf, i + 5)
            if length == 0 or length > MAX_FRAME:
                del buf[: i + 1]  # bogus header, resync
                continue
            if len(buf) < i + 9 + length:
                break
            jpg = bytes(buf[i + 9 : i + 9 + length])
            del buf[: i + 9 + length]
            yield jpg


def main():
    ser = serial.Serial(PORT, 921600, timeout=1)
    print(f"Connected to {PORT}. Press q in the window to quit.")
    last, fps = time.time(), 0.0
    for jpg in frames(ser):
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue
        now = time.time()
        fps = 0.9 * fps + 0.1 * (1 / max(now - last, 1e-6))
        last = now
        cv2.putText(img, f"{fps:.1f} fps", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("ESP32-S3 Camera", img)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
