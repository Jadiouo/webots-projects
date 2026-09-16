from controller import Supervisor
import numpy as np

class ArmRLController:
    def __init__(self):
        # 初始化 Supervisor 以讀取世界座標訊息 
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # 1. 取得裝置與傳感器 [cite: 91, 262]
        self.motors = []
        self.sensors = []
        for i in range(1, 4):
            m = self.robot.getDevice(f'motor{i}')
            s = self.robot.getDevice(f'ps{i}')
            s.enable(self.timestep)
            self.motors.append(m)
            self.sensors.append(s)
            
        # 2. 定義目標點 A, B, C (對應 DEF 名稱) [cite: 23, 191]
        self.target_names = ['TARGET_A', 'TARGET_B', 'TARGET_C']
        self.targets = [self.robot.getFromDef(name) for name in self.target_names]
        self.current_idx = 0
        
        # 3. 取得末端效果器節點 
        self.ee_node = self.robot.getFromDef('EE_POINT')
        
    def get_observation(self):
        """ 觀察值 (State): 包含關節位置與目標點的 X-Z 座標 """
        # 讀取馬達當前位置 [cite: 135]
        joint_pos = [s.getValue() for s in self.sensors]
        
        # 讀取當前目標球體的位置 (世界座標) [cite: 22, 191]
        target_pos = self.targets[self.current_idx].getField('translation').getSFVec3f()
        
        # 回傳向量: [ps1, ps2, ps3, target_x, target_z]
        return np.array(joint_pos + [target_pos[0], target_pos[2]])

    def compute_reward(self):
        """ 獎勵函數 (Reward): 越靠近目標獎勵越高 """
        # 取得末端點的世界座標 [cite: 22]
        ee_pos = self.ee_node.getPosition()
        # 取得目標點的世界座標 [cite: 22]
        target_pos = self.targets[self.current_idx].getField('translation').getSFVec3f()
        
        # 計算 X-Z 平面上的距離 (Y 高度固定為 0.2m) [cite: 22, 107]
        dist = np.linalg.norm(np.array([ee_pos[0], ee_pos[2]]) - np.array([target_pos[0], target_pos[2]]))
        
        reward = -dist  # 基礎負獎勵：距離越遠分數越低
        reached = False
        
        # 到點判斷：若距離小於閾值則視為達標 [cite: 135, 192]
        if dist < 0.03:
            reward += 100.0
            reached = True
            print(f"到達目標 {self.target_names[self.current_idx]}!")
            # 切換至下一個目標點 (A->B->C->A 循環) [cite: 3, 104]
            self.current_idx = (self.current_idx + 1) % 3
            
        return reward, reached

    def apply_action(self, action):
        """ 執行動作 (Action): 給予馬達位置增量 """
        # action 預期為三個數值的陣列，代表 motor1, motor2, motor3 的位置增量
        for i, motor in enumerate(self.motors):
            current_val = self.sensors[i].getValue()
            new_pos = current_val + action[i]
            
            # 對 motor2 (線性馬達) 進行範圍限制 (maxStop 0.3) [cite: 93, 218]
            if i == 1:
                new_pos = max(0.0, min(0.3, new_pos))
                
            motor.setPosition(new_pos)

    def run(self):
        """ 訓練/運行主迴圈 """
        while self.robot.step(self.timestep) != -1:
            # A. 獲取當前觀察值
            obs = self.get_observation()
            
            # B. 決定動作 (此處應替換為你的強化學習模型輸出)
            # 現在使用微小的隨機擾動進行測試
            random_action = np.random.uniform(-0.01, 0.01, 3)
            
            # C. 執行動作
            self.apply_action(random_action)
            
            # D. 獲取獎勵與反饋
            reward, reached = self.compute_reward()

# 執行控制器
if __name__ == "__main__":
    controller = ArmRLController()
    controller.run()