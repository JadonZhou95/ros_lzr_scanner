import lzr_scanner as lzr
import time

port = '/dev/ttyUSB0'
baud_rate = 460800


if __name__ == '__main__':
    scan = lzr.LzrScanner(port, baud_rate)
    scan.connect()

    capture_count = 0
    start_time = time.time()
    while True:
        if not scan.capture_frame():
            capture_count += 1
            # scan.visualize_scan()

        # count the image captured within one second
        if time.time() - start_time > 1:
            print("Hz: " + str(capture_count))
            capture_count = 0
            start_time = time.time()
