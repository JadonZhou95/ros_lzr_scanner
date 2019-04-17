import lzr_scanner as lzr

if __name__ == '__main__':
    scan = lzr.LzrScanner('/dev/ttyUSB0', 460800)
    scan.connect()
    while True:
        if not scan.capture_frame():
            pass
            # scan.visualize_scan()