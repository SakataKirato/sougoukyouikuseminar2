import socket
import numpy as np
import cv2
from collections import deque
import struct
import serial

PORT = 5005
W, H = 160, 120

# ROI設定（ライン検出範囲）
# カーブ対応: 狭めると直近の進路に敏感、広めると安定
ROI_TOP_RATIO = 0.4     # 上部カット率（0.0-1.0）
ROI_BOTTOM_RATIO = 0.9   # 下部カット率（0.0-1.0）

# 角度スムージング設定
ANGLE_SMOOTH_WINDOW = 10  # 移動平均のウィンドウサイズ（大きいほど滑らか）

# メカナムホイール制御パラメータ
BASE_SPEED = 100     # 基本前進速度 (0-255)
Kp_x = 0.5          # 横方向位置補正ゲイン
Kp_angle = 0.5      # 角度補正ゲイン
MAX_SPEED = 255     # モーター最大速度
MIN_SPEED = -255    # モーター最小速度

# シリアル通信設定
SERIAL_PORT = '/dev/ttyUSB1'  # シリアルポート（環境に応じて変更）
SERIAL_BAUD = 115200          # ボーレート
SERIAL_ENABLE = True        # シリアル送信を有効化（Trueで送信開始）

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))

frames = {}
angle_history = deque(maxlen=ANGLE_SMOOTH_WINDOW)  # 角度履歴バッファ

# シリアルポート初期化
ser = None
if SERIAL_ENABLE:
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
        print(f"シリアルポート {SERIAL_PORT} を開きました (ボーレート: {SERIAL_BAUD})")
    except Exception as e:
        print(f"シリアルポートのオープンに失敗: {e}")
        print("シリアル送信は無効化されます")
        SERIAL_ENABLE = False

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
        # 2. ガウシアンブラー（ノイズ除去）
        # =========================
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # =========================
        # 3. 適応的二値化
        # =========================
        bw = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            201,  # blockSize を小さく（より細かい適応）
            5    # C を小さく（より敏感に）
        )

        # =========================
        # 4. ノイズ除去（OPEN + CLOSE）
        # =========================
        kernel = np.ones((2, 2), np.uint8)
        bw_clean = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel)
        # CLOSE処理で線の途切れを補完
        bw_clean = cv2.morphologyEx(bw_clean, cv2.MORPH_CLOSE, kernel)

        # =========================
        # 4.5. ROI設定（上下端を除外）
        # =========================
        roi_top = int(H * ROI_TOP_RATIO)
        roi_bottom = int(H * ROI_BOTTOM_RATIO)
        bw_roi = bw_clean[roi_top:roi_bottom, :]

        # =========================
        # 5. 輪郭検出（黒線）- ROI内のみ
        # =========================
        bw_inv = cv2.bitwise_not(bw_roi)

        contours, _ = cv2.findContours(
            bw_inv,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # =========================
        # 6. 線（最大輪郭）だけ描画
        # =========================
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        if contours:
            # 面積でフィルタリング（小さすぎるノイズを除外）
            min_area = 100  # 最小面積の閾値
            valid_contours = [c for c in contours if cv2.contourArea(c) > min_area]
            
            if valid_contours:
                # 面積最大の輪郭 = ライン
                line_contour = max(valid_contours, key=cv2.contourArea)
                
                # ROI座標をオリジナル画像座標に変換
                line_contour_adjusted = line_contour.copy()
                line_contour_adjusted[:, :, 1] += roi_top
                
                cv2.drawContours(vis, [line_contour_adjusted], -1, (0, 0, 255), 2)
                
                # 直線フィッティングで傾きを計算
                [vx, vy, x, y] = cv2.fitLine(line_contour_adjusted, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # 傾き（ラジアン → 度）
                # 画像座標系はY軸が下向きなので、vyを反転して垂直上向きを0度とする
                angle_rad = np.arctan2(-vy, vx)  # vyを反転
                angle_deg_raw = np.degrees(angle_rad)[0] - 90.0
                
                # 角度スムージング（移動平均）
                angle_history.append(angle_deg_raw)
                angle_deg = np.mean(angle_history)
                
                # 角度を-90〜+90度に正規化
                if angle_deg > 90:
                    angle_deg -= 180
                elif angle_deg < -90:
                    angle_deg += 180
                
                # 重心を計算して表示（元の画像座標系で計算）
                M = cv2.moments(line_contour_adjusted)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(vis, (cx, cy), 3, (0, 255, 0), -1)
                    
                    # 傾きを表示（スムージング済み + 生データ）
                    cv2.putText(vis, f"Angle: {angle_deg:.1f} deg (raw: {angle_deg_raw:.1f})", 
                                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                    
                    # フィッティングした直線を描画（デバッグ用）
                    lefty = int((-x * vy / vx) + y)
                    righty = int(((W - x) * vy / vx) + y)
                    cv2.line(vis, (W-1, righty), (0, lefty), (255, 0, 0), 1)
                    
                    # =========================
                    # メカナムホイール速度計算
                    # =========================
                    # 誤差計算
                    error_angle = angle_deg - 0.0    # 角度誤差（0度が目標）
                    error_x = cx - (W / 2)           # 位置誤差（画面中心が目標）
                    
                    # 比例制御で補正量を計算
                    correction_x = Kp_x * error_x
                    correction_angle = Kp_angle * error_angle
                    
                    # 速度成分
                    vx_speed = -correction_x        # 横移動（左右補正）
                    vy_speed = BASE_SPEED          # 前進
                    omega = -correction_angle      # 回転（角度補正）
                    
                    # メカナムホイール逆運動学
                    FL = vy_speed - vx_speed - omega  # 前左
                    FR = vy_speed + vx_speed + omega  # 前右
                    BL = vy_speed + vx_speed - omega  # 後左
                    BR = vy_speed - vx_speed + omega  # 後右
                    
                    # 速度の正規化（範囲制限）
                    speeds = [FL, FR, BL, BR]
                    max_abs = max(abs(s) for s in speeds)
                    
                    if max_abs > MAX_SPEED:
                        scale = MAX_SPEED / max_abs
                        FL *= scale
                        FR *= scale
                        BL *= scale
                        BR *= scale
                    
                    # 範囲制限
                    FL = int(max(MIN_SPEED, min(MAX_SPEED, FL)))
                    FR = int(max(MIN_SPEED, min(MAX_SPEED, FR)))
                    BL = int(max(MIN_SPEED, min(MAX_SPEED, BL)))
                    BR = int(max(MIN_SPEED, min(MAX_SPEED, BR)))
                    
                    # デバッグ出力
                    serial_status = "[SENT]" if SERIAL_ENABLE and ser else "[NO SERIAL]"
                    print(f"{serial_status} Angle:{angle_deg:+6.1f}° ErrA:{error_angle:+6.1f}° | "
                          f"Pos:({cx:3d},{cy:3d}) ErrX:{error_x:+4.0f} | "
                          f"Motors: FL={FL:+4d} FR={FR:+4d} BL={BL:+4d} BR={BR:+4d}")
                    
                    # シリアル送信
                    if SERIAL_ENABLE and ser:
                        try:
                            # 4つのshort（2バイト整数）としてパック
                            data = struct.pack('hhhh', FL, FR, BL, BR)
                            ser.write(data)
                        except Exception as e:
                            print(f"シリアル送信エラー: {e}")

        # ROI境界を表示（デバッグ用）
        cv2.line(vis, (0, roi_top), (W-1, roi_top), (255, 255, 0), 1)
        cv2.line(vis, (0, roi_bottom), (W-1, roi_bottom), (255, 255, 0), 1)

        # =========================
        # 7. 表示
        # =========================
        scale = 4
        vis_show = cv2.resize(
            vis, (W*scale, H*scale),
            interpolation=cv2.INTER_NEAREST
        )
        
        # 二値化結果も表示(デバッグ用)
        bw_show = cv2.resize(
            bw_clean, (W*scale, H*scale),
            interpolation=cv2.INTER_NEAREST
        )

        cv2.imshow("line contour only", vis_show)
        cv2.imshow("binary (debug)", bw_show)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
