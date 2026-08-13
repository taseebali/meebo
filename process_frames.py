#!/usr/bin/env python3
import sys
import os
import shutil
import socket
import concurrent.futures

# Ensure UTF-8 output encoding for Windows terminal
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

# Try importing OpenCV and NumPy, print friendly guide if missing
try:
    import cv2
    import numpy as np
except ImportError:
    print("❌ Missing required libraries: cv2 or numpy.")
    print("Please install them by running: py -3 -m pip install opencv-python numpy")
    sys.exit(1)

try:
    import paramiko
except ImportError:
    paramiko = None

# Directories
WORKSPACE_DIR = os.path.dirname(os.path.abspath(__file__))
FRAMES_DIR = os.path.join(WORKSPACE_DIR, 'lane_frames')
MASKS_DIR = os.path.join(WORKSPACE_DIR, 'lane_frames_masks')

# Current tuned HSV & ROI Parameters
HSV_LOWER = np.array([0, 0, 0])
HSV_UPPER = np.array([180, 255, 110])
ROI_TOP = 300
ROI_BOTTOM = 700
MIN_CONTOUR_AREA = 200

PI_USER = 'etw3'
PI_PASS = 'etw3-team03'
KNOWN_IPS = ['192.168.137.86', '192.168.137.71', '192.168.137.162', '192.168.137.184', '192.168.137.205', '192.168.137.41']


def contour_center_x(contour):
    m = cv2.moments(contour)
    return m['m10'] / m['m00'] if m['m00'] > 0 else 0


def download_remote_frames():
    if paramiko is None:
        print("❌ Paramiko library missing. Install it using: py -3 -m pip install paramiko")
        sys.exit(1)

    print("🔄 [MODE: NEW] Clearing old local frames and downloading fresh frames from Raspberry Pi...")
    
    if os.path.exists(FRAMES_DIR):
        shutil.rmtree(FRAMES_DIR)
    os.makedirs(FRAMES_DIR, exist_ok=True)

    def try_sftp_download(ip):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.5)
            if sock.connect_ex((ip, 22)) == 0:
                sock.close()
                client = paramiko.SSHClient()
                client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                client.connect(ip, username=PI_USER, password=PI_PASS, timeout=2)
                
                sftp = client.open_sftp()
                files = sftp.listdir('/home/etw3/lane_frames')
                print(f"✅ Connected to Pi at {ip}. Found {len(files)} files in ~/lane_frames.")
                
                for f in files:
                    remote_path = f'/home/etw3/lane_frames/{f}'
                    local_path = os.path.join(FRAMES_DIR, f)
                    sftp.get(remote_path, local_path)
                
                sftp.close()
                client.close()
                return True
        except Exception:
            pass
        return False

    found = False
    for ip in KNOWN_IPS:
        if try_sftp_download(ip):
            found = True
            break

    if not found:
        with concurrent.futures.ThreadPoolExecutor(max_workers=50) as executor:
            futures = [executor.submit(try_sftp_download, f"192.168.137.{i}") for i in range(1, 255)]
            for f in concurrent.futures.as_completed(futures):
                if f.result():
                    found = True
                    break

    if not found:
        print("⚠️ Could not connect to Raspberry Pi over SFTP. Proceeding with existing local frames (if any).")


def process_masks():
    if not os.path.exists(FRAMES_DIR):
        os.makedirs(FRAMES_DIR, exist_ok=True)

    frame_files = [f for f in os.listdir(FRAMES_DIR) if f.endswith('.png') and not f.endswith('_mask.png')]
    frame_files.sort()

    if not frame_files:
        print(f"⚠️ No image files found in {FRAMES_DIR}.")
        print("Tip: Run 'python process_frames.py new' to fetch frames from the Raspberry Pi.")
        return

    os.makedirs(MASKS_DIR, exist_ok=True)
    print(f"\n🖼️ Processing {len(frame_files)} frames from {FRAMES_DIR}...")

    report_lines = [
        "# Lane Frames Mask Processing Report\n",
        f"Processed **{len(frame_files)} frames** using tuned parameters:\n",
        f"- **HSV Range**: `Lower = {HSV_LOWER.tolist()}`, `Upper = {HSV_UPPER.tolist()}`\n",
        f"- **ROI Crop**: `Rows {ROI_TOP} to {ROI_BOTTOM}`\n",
        f"- **Min Contour Area**: `{MIN_CONTOUR_AREA}` pixels\n\n",
        "| Frame File | Status / Tape Detection | Midpoint Offset |\n",
        "| :--- | :--- | :--- |\n"
    ]

    for filename in frame_files:
        frame_path = os.path.join(FRAMES_DIR, filename)
        frame = cv2.imread(frame_path)
        if frame is None:
            continue

        height, width = frame.shape[:2]
        top = max(0, min(ROI_TOP, height))
        bottom = max(top, min(ROI_BOTTOM, height))

        roi = frame[top:bottom, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        significant = sorted(
            (c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA),
            key=cv2.contourArea,
            reverse=True
        )[:2]

        significant.sort(key=contour_center_x)

        left_tape = significant[0] if len(significant) >= 1 else None
        right_tape = significant[1] if len(significant) >= 2 else None

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Draw ROI Box & Center Guide
        cv2.rectangle(frame, (0, top), (width, bottom), (0, 255, 255), 2)
        frame_center_x = width / 2.0
        cv2.line(frame, (int(frame_center_x), top), (int(frame_center_x), bottom), (255, 0, 0), 2)

        status_str = "No Tape"
        offset_val = 0.0

        if left_tape is not None and right_tape is not None:
            lx = contour_center_x(left_tape)
            rx = contour_center_x(right_tape)
            mid_x = (lx + rx) / 2.0
            offset_val = (mid_x - frame_center_x) / frame_center_x
            
            cv2.drawContours(frame[top:bottom, :], [left_tape], -1, (0, 255, 0), 3)
            cv2.drawContours(frame[top:bottom, :], [right_tape], -1, (0, 0, 255), 3)
            cv2.circle(frame, (int(mid_x), int((top + bottom)/2)), 6, (0, 255, 255), -1)
            status_str = f"2 Tapes Midpoint (offset={offset_val:+.3f})"
        elif left_tape is not None:
            lx = contour_center_x(left_tape)
            offset_val = (lx - frame_center_x) / frame_center_x
            cv2.drawContours(frame[top:bottom, :], [left_tape], -1, (0, 255, 0), 3)
            status_str = f"1 Tape (offset={offset_val:+.3f})"

        # Create combined side-by-side view (400x300 scaled)
        frame_resized = cv2.resize(frame, (400, 300))
        mask_resized = cv2.resize(mask_bgr, (400, 300))
        combined = np.hstack((frame_resized, mask_resized))
        cv2.putText(combined, f"{filename}: {status_str}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        mask_filename = filename.replace('.png', '_mask.png')
        mask_out_path = os.path.join(MASKS_DIR, mask_filename)
        cv2.imwrite(mask_out_path, combined)

        raw_mask_path = os.path.join(MASKS_DIR, f"raw_{mask_filename}")
        cv2.imwrite(raw_mask_path, mask)

        report_lines.append(f"| `{filename}` | {status_str} | `{offset_val:+.3f}` |\n")

    report_path = os.path.join(WORKSPACE_DIR, 'lane_frames_report.md')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.writelines(report_lines)

    print(f"✅ Generated {len(frame_files)} mask files in: {MASKS_DIR}")
    print(f"📝 Generated summary report at: {report_path}")


def main():
    args = [a.lower() for a in sys.argv[1:]]
    
    if 'new' in args or 'fetch' in args or '-n' in args:
        download_remote_frames()
    
    process_masks()


if __name__ == '__main__':
    main()
