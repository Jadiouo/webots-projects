from controller import Robot
import math

TIME_STEP = 16
MAX_V   = 10.0
BASE_V  = 7.0
GAIN    = 12.0
NL      = 3.0
SLOW_K  = 0.7
DEADBAND = 0.015
ALPHA    = 0.3

# ★ 近距參數：靠近光源時（總亮度大）提高轉向、同時更慢
NEAR_GAIN   = 3.0    # 近光的額外轉向倍率（2~6 調）
NEAR_SLOW   = 0.6    # 近光的降速強度（0~1）
NEAR_S_THR  = 0.02   # 認定「靠近」的大致閾值（看你的 L+R 量級調）

def clamp(x, lo, hi): return hi if x > hi else lo if x < lo else x

r = Robot()

# 四個馬達
lf = r.getDevice('left_front_motor')
lr = r.getDevice('left_rear_motor')
rf = r.getDevice('right_front_motor')
rr = r.getDevice('right_rear_motor')
for m in (lf, lr, rf, rr):
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

# 兩顆光感測器
lls = r.getDevice('left_ls');  lls.enable(TIME_STEP)
rls = r.getDevice('right_ls'); rls.enable(TIME_STEP)

# 啟動校正
avg_n = int(800 / TIME_STEP)
la = ra = 0.0
for _ in range(avg_n):
    if r.step(TIME_STEP) == -1: break
    la += lls.getValue(); ra += rls.getValue()
L0 = la / max(1, avg_n); R0 = ra / max(1, avg_n)

steer_f = 0.0

while r.step(TIME_STEP) != -1:
    L = max(0.0, lls.getValue() - L0)
    R = max(0.0, rls.getValue() - R0)

    s = L + R + 1e-6
    err = (L - R) / s              # 左亮=正、右亮=負
    if abs(err) < DEADBAND:
        err = 0.0

    # 平滑
    steer_f = (1 - ALPHA) * steer_f + ALPHA * err

    # ★ 近距增益：總亮度越大，轉向倍率越高（把小差拉大）
    #   s_norm ∈ [0,1]；s 達到 NEAR_S_THR 時視為「近」
    s_norm = min(1.0, s / NEAR_S_THR)
    near_mul = 1.0 + NEAR_GAIN * s_norm

    # 非線性放大 + 近距增益
    steer_nl = math.tanh(NL * steer_f * near_mul)

    # ★ 近距減速：越近越慢，轉得更過去（避免貼近時直衝）
    base = BASE_V * (1.0 - SLOW_K * abs(steer_nl))
    base *= (1.0 - NEAR_SLOW * s_norm)   # 近光再降速

    turn = GAIN * steer_nl

    vL = clamp(base - turn, -MAX_V, MAX_V)
    vR = clamp(base + turn, -MAX_V, MAX_V)

    lf.setVelocity(vL); lr.setVelocity(vL)
    rf.setVelocity(vR); rr.setVelocity(vR)

    # 偵錯列印（可關）
    if r.getTime() % 0.25 < (TIME_STEP/1000.0):
        print(f"L={L:.4f} R={R:.4f} s={s:.4f} sN={s_norm:.2f} err={err:.4f} nl={steer_nl:.3f} base={base:.2f} vL={vL:.2f} vR={vR:.2f}")
