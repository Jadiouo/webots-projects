from controller import Robot
import math

robot = Robot()
ts = int(robot.getBasicTimeStep())
print("[set_pose] started")

def M(name):
    m = robot.getDevice(name)
    if m: m.setVelocity(1.5)
    return m

m1 = M('motor1'); m2 = M('motor2'); m3 = M('motor3')

def step_for(sec):
    for _ in range(int(sec*1000/ts)):
        robot.step(ts)

# 先歸零，確保看得到動作
for m in (m1, m2, m3):
    if m: m.setPosition(0.0)
step_for(1.2)

# 轉到 (30°, -20°, 45°)
if m1: m1.setPosition(math.radians(70))
if m2: m2.setPosition(math.radians(-50))
if m3: m3.setPosition(math.radians(90))
step_for(2.0)

# 之後保持運行（可加微擺動測試）
while robot.step(ts) != -1:
    pass
