"""
メカナムホイール4輪ライントレースロボット制御プログラム
角度+位置ハイブリッド制御方式

制御データ受信フォーマット:
- cx (short): 線の重心X座標
- cy (short): 線の重心Y座標
- angle (float): 線の角度（度）
- error_x (short): 画面中心からのX偏差
- error_y (short): 画面中心からのY偏差
"""

import socket
import struct
import time

# =========================
# 設定パラメータ
# =========================
CONTROL_PORT = 5006
BASE_SPEED = 100  # 基本前進速度 (0-255)

# PIDパラメータ（要チューニング）
Kp_x = 2.0      # 横方向位置補正ゲイン
Kp_angle = 1.5  # 角度補正ゲイン
Kd_x = 0.5      # 横方向微分ゲイン
Kd_angle = 0.3  # 角度微分ゲイン

TARGET_ANGLE = 90.0  # 目標角度（90度=垂直上向き）

# モータ速度制限
MAX_SPEED = 255
MIN_SPEED = -255

# =========================
# メカナムホイール運動学
# =========================
def mecanum_kinematics(vx, vy, omega):
    """
    メカナムホイールの運動学計算
    
    引数:
        vx: X方向速度（横移動、右が正）
        vy: Y方向速度（前進、前が正）
        omega: 回転速度（反時計回りが正）
    
    戻り値:
        (FL, FR, BL, BR): 各モータの速度
        FL: 前左, FR: 前右, BL: 後左, BR: 後右
    """
    # メカナムホイール逆運動学
    FL = vy - vx - omega
    FR = vy + vx + omega
    BL = vy + vx - omega
    BR = vy - vx + omega
    
    return FL, FR, BL, BR

def normalize_speeds(FL, FR, BL, BR):
    """
    モータ速度を正規化して範囲内に収める
    """
    speeds = [FL, FR, BL, BR]
    max_abs = max(abs(s) for s in speeds)
    
    if max_abs > MAX_SPEED:
        scale = MAX_SPEED / max_abs
        speeds = [s * scale for s in speeds]
    
    # 範囲制限
    speeds = [max(MIN_SPEED, min(MAX_SPEED, s)) for s in speeds]
    
    return tuple(map(int, speeds))

# =========================
# 制御ループ
# =========================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", CONTROL_PORT))
sock.settimeout(0.5)  # タイムアウト設定

print(f"制御データ受信中... (ポート: {CONTROL_PORT})")
print("Ctrl+C で終了")

# 前回の偏差（微分計算用）
prev_error_x = 0
prev_error_angle = 0
prev_time = time.time()

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            
            # データ解析
            cx, cy, angle_deg, error_x, error_y = struct.unpack('hhfhh', data)
            
            # 時間差分計算
            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time
            
            # 角度偏差計算
            error_angle = angle_deg - TARGET_ANGLE
            
            # 角度を-90〜90度に正規化
            if error_angle > 90:
                error_angle -= 180
            elif error_angle < -90:
                error_angle += 180
            
            # 微分項計算
            if dt > 0:
                d_error_x = (error_x - prev_error_x) / dt
                d_error_angle = (error_angle - prev_error_angle) / dt
            else:
                d_error_x = 0
                d_error_angle = 0
            
            # PD制御計算
            correction_x = Kp_x * error_x + Kd_x * d_error_x
            correction_angle = Kp_angle * error_angle + Kd_angle * d_error_angle
            
            # メカナムホイール制御
            # vx: 横移動（error_xを補正）
            # vy: 前進（固定速度）
            # omega: 回転（角度を補正）
            vx = -correction_x  # 左右補正（符号注意）
            vy = BASE_SPEED     # 前進
            omega = -correction_angle  # 回転補正
            
            # モータ速度計算
            FL, FR, BL, BR = mecanum_kinematics(vx, vy, omega)
            FL, FR, BL, BR = normalize_speeds(FL, FR, BL, BR)
            
            # デバッグ出力
            print(f"X:{cx:3d} Y:{cy:3d} Ang:{angle_deg:5.1f}° | "
                  f"ErrX:{error_x:+4d} ErrA:{error_angle:+5.1f}° | "
                  f"Motors: FL={FL:+4d} FR={FR:+4d} BL={BL:+4d} BR={BR:+4d}")
            
            # ここでモータに送信する処理を追加
            # 例: serial.write() や別のUDP送信など
            # motor_command = struct.pack('hhhh', FL, FR, BL, BR)
            # motor_sock.sendto(motor_command, (ROBOT_IP, MOTOR_PORT))
            
            # 前回値を保存
            prev_error_x = error_x
            prev_error_angle = error_angle
            
        except socket.timeout:
            print("タイムアウト: 制御データが受信できません")
            # タイムアウト時は停止
            FL = FR = BL = BR = 0
            
except KeyboardInterrupt:
    print("\n制御を終了します")
finally:
    sock.close()
