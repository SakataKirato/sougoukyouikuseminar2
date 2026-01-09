import socket
import numpy as np
import cv2

PORT = 5005
W, H = 160, 120

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))

frames = {}

print("UDP receiving...")

while True:
    data, addr = sock.recvfrom(2048)

    # --- ヘッダ解析 ---
    frame_id = (data[0] << 8) | data[1]
    chunk_id = (data[2] << 8) | data[3]
    total    = (data[4] << 8) | data[5]
    payload  = data[6:]

    if frame_id not in frames:
        frames[frame_id] = [None] * total

    frames[frame_id][chunk_id] = payload

    # --- フレームが揃ったら ---
    if all(p is not None for p in frames[frame_id]):
        raw = b"".join(frames[frame_id])
        del frames[frame_id]

        # =========================
        # 1. RAW → グレースケール
        # =========================
        gray = np.frombuffer(raw, np.uint8).reshape(H, W)

        # =========================
        # 2. 適応的二値化
        # =========================
        bw = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            21,
            5
        )

        # =========================
        # 3. ノイズ除去（OPEN）
        # =========================
        kernel = np.ones((2, 2), np.uint8)
        bw_clean = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel)

        # =========================
        # 4. 輪郭検出（黒線）
        # =========================
        bw_inv = cv2.bitwise_not(bw_clean)

        contours, _ = cv2.findContours(
            bw_inv,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # =========================
        # 5. 線（最大輪郭）だけ描画
        # =========================
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        if contours:
            # 面積最大の輪郭 = ライン
            line_contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(vis, [line_contour], -1, (0, 0, 255), 2)

        # =========================
        # 6. 表示
        # =========================
        scale = 4
        vis_show = cv2.resize(
            vis, (W*scale, H*scale),
            interpolation=cv2.INTER_NEAREST
        )

        cv2.imshow("line contour only", vis_show)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
