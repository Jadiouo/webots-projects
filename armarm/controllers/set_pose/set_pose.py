# set_pose.py
# Controller for 3-DOF R-P-R planar robot arm in Webots
#
# Mode 0: Joint-space keyboard control
#   Q/A : theta1 +/- 
#   W/S : d2     +/- 
#   E/D : theta3 +/-
#
# Mode 1: Task-space keyboard control (end-effector)
#   J/L : x_target - / +
#   I/K : y_target + / -
#   U/O : theta_target + / -
#
#   M   : toggle mode between JOINT and TASK
#
# Prints every few steps (not every step):
#   theta1, d2, theta3, end-effector (x, y), orientation theta

from controller import Robot, Keyboard
import math

# -----------------------------
# Configurable parameters
# -----------------------------

TIME_STEP = 32  # will be overwritten by robot.getBasicTimeStep()

# 幾何長度 (以目前 world 尺寸為例，可再調整)
L1 = 0.24  # 第一段 link 長度
L2 = 0.24  # 第二段 link 長度
L3 = 0.24  # 第三段 link 長度（末端那段）

# 控制增量
DELTA_THETA = 0.02    # 每次按鍵增減的角度 (rad)
DELTA_D2 = 0.005      # 每次按鍵增減的平移量 (m)
DELTA_XY = 0.01       # 末端在 x,y 方向的每次移動量 (m)

# d2 限制 (要跟 world 裡 LinearMotor 的 minPosition / maxPosition 對應)
D2_MIN = 0.0
D2_MAX = 0.24

# 為了避免「瞬間大跳躍」，每一步只走這麼多
MAX_STEP_THETA = 0.05   # rad
MAX_STEP_D2 = 0.01      # m

MODE_JOINT = 0
MODE_TASK = 1

# ✅ 新增：控制輸出頻率（例如每 10 步印一次）
PRINT_EVERY = 10

# -----------------------------
# Forward kinematics
# -----------------------------
def forward_kinematics(theta1, d2, theta3):
    """
    簡化版 R-P-R 平面手臂的正運動學
    """
    r1 = L1 + d2 + L2
    x = r1 * math.cos(theta1) + L3 * math.cos(theta1 + theta3)
    y = r1 * math.sin(theta1) + L3 * math.sin(theta1 + theta3)
    theta = theta1 + theta3
    return x, y, theta


# -----------------------------
# Inverse kinematics
# -----------------------------
def inverse_kinematics(x, y, theta_end):
    """
    解析式 IK：
    x' = x - L3*cos(theta_end)
    y' = y - L3*sin(theta_end)
    r1 = sqrt(x'^2 + y'^2)
    theta1 = atan2(y', x')
    d2 = r1 - (L1 + L2)
    theta3 = theta_end - theta1
    """
    x_p = x - L3 * math.cos(theta_end)
    y_p = y - L3 * math.sin(theta_end)

    r1 = math.hypot(x_p, y_p)
    if r1 < 1e-6:
        return None

    theta1 = math.atan2(y_p, x_p)
    d2 = r1 - (L1 + L2)

    if d2 < D2_MIN or d2 > D2_MAX:
        return None

    theta3 = theta_end - theta1

    theta1 = (theta1 + math.pi) % (2 * math.pi) - math.pi
    theta3 = (theta3 + math.pi) % (2 * math.pi) - math.pi

    return theta1, d2, theta3


# -----------------------------
# Main controller
# -----------------------------
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

kb = Keyboard()
kb.enable(TIME_STEP)

motor1 = robot.getDevice("motor1")
motor2 = robot.getDevice("motor2")
motor3 = robot.getDevice("motor3")

ps1 = robot.getDevice("ps1")
ps2 = robot.getDevice("ps2")
ps3 = robot.getDevice("ps3")
ps1.enable(TIME_STEP)
ps2.enable(TIME_STEP)
ps3.enable(TIME_STEP)

robot.step(TIME_STEP)
theta1_cmd = ps1.getValue()
d2_cmd = ps2.getValue()
theta3_cmd = ps3.getValue()

motor1.setPosition(theta1_cmd)
motor2.setPosition(d2_cmd)
motor3.setPosition(theta3_cmd)

x_target, y_target, theta_target = forward_kinematics(theta1_cmd, d2_cmd, theta3_cmd)

mode = MODE_JOINT

# ✅ 新增：步數計數器
step_counter = 0

print("Keyboard control ready!")
print("Mode 0 = JOINT, Mode 1 = TASK")
print("Switch mode: M")
print("JOINT MODE: Q/A theta1 +/- | W/S d2 +/- | E/D theta3 +/-")
print("TASK  MODE: J/L x -/+      | I/K y +/-  | U/O theta +/-")
print("--------------------------------------------------------")

while robot.step(TIME_STEP) != -1:
    step_counter += 1

    key = kb.getKey()
    while key != -1:
        if key == ord('M'):
            mode = MODE_TASK if mode == MODE_JOINT else MODE_JOINT
            mode_name = "TASK (end-effector)" if mode == MODE_TASK else "JOINT (angles)"
            print(f"== Switch to {mode_name} mode ==")

        if mode == MODE_JOINT:
            if key == ord('Q'):
                theta1_cmd += DELTA_THETA
            elif key == ord('A'):
                theta1_cmd -= DELTA_THETA
            elif key == ord('W'):
                d2_cmd += DELTA_D2
            elif key == ord('S'):
                d2_cmd -= DELTA_D2
            elif key == ord('E'):
                theta3_cmd += DELTA_THETA
            elif key == ord('D'):
                theta3_cmd -= DELTA_THETA
        else:
            if key == ord('L'):
                x_target += DELTA_XY
            elif key == ord('J'):
                x_target -= DELTA_XY
            elif key == ord('I'):
                y_target += DELTA_XY
            elif key == ord('K'):
                y_target -= DELTA_XY
            elif key == ord('U'):
                theta_target += DELTA_THETA
            elif key == ord('O'):
                theta_target -= DELTA_THETA

        key = kb.getKey()

    if mode == MODE_TASK:
        ik_solution = inverse_kinematics(x_target, y_target, theta_target)
        if ik_solution is not None:
            theta1_ik, d2_ik, theta3_ik = ik_solution

            def step_towards(current, target, max_step):
                delta = target - current
                if delta > max_step:
                    delta = max_step
                elif delta < -max_step:
                    delta = -max_step
                return current + delta

            theta1_cmd = step_towards(theta1_cmd, theta1_ik, MAX_STEP_THETA)
            d2_cmd = step_towards(d2_cmd, d2_ik, MAX_STEP_D2)
            theta3_cmd = step_towards(theta3_cmd, theta3_ik, MAX_STEP_THETA)
        else:
            # 無解的時候還是立刻顯示一下
            print(f"[IK] no solution for target x={x_target:.3f}, "
                  f"y={y_target:.3f}, theta={theta_target:.3f} rad")

    if d2_cmd < D2_MIN:
        d2_cmd = D2_MIN
    if d2_cmd > D2_MAX:
        d2_cmd = D2_MAX

    motor1.setPosition(theta1_cmd)
    motor2.setPosition(d2_cmd)
    motor3.setPosition(theta3_cmd)

    theta1 = ps1.getValue()
    d2 = ps2.getValue()
    theta3 = ps3.getValue()

    x, y, theta = forward_kinematics(theta1, d2, theta3)

    mode_str = "JOINT" if mode == MODE_JOINT else "TASK"

    # ✅ 只在每 PRINT_EVERY 步時印一次狀態
    if step_counter % PRINT_EVERY == 0:
        print(
            f"[{mode_str}] "
            f"θ1 = {theta1: .3f} rad ({math.degrees(theta1): .1f} deg), "
            f"d2 = {d2: .3f} m, "
            f"θ3 = {theta3: .3f} rad ({math.degrees(theta3): .1f} deg) | "
            f"end (x, y) = ({x: .3f}, {y: .3f}), "
            f"θ = {theta: .3f} rad ({math.degrees(theta): .1f} deg)"
        )
