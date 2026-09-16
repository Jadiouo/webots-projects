# webots-projects

Webots（R2025a）機器人模擬專案集合，全部使用 Python controller。

每個子資料夾都是一個獨立的 Webots 專案（`worlds/` + `controllers/`），用 Webots 開啟對應的 `.wbt` 即可執行。

| 專案 | World | Controller | 說明 |
|---|---|---|---|
| `hw1/` | `hw1.wbt` | `light_follower` | 四輪車 + 兩顆光感測器的趨光控制：以左右亮度差做非線性轉向，靠近光源時提高轉向增益並減速 |
| `armarm/` | `arm.wbt` | `set_pose` | 3-DOF R-P-R 平面手臂：鍵盤控制，可在關節空間（Q/A、W/S、E/D）與任務空間（J/L、I/K、U/O，內含逆運動學）之間用 `M` 切換，並印出關節值與末端位姿 |
| `robotarm/` | `robot_arm.wbt` | `set_pose` | 手臂設定固定姿態的最小範例 |
| `hw5/` | `hw5.wbt` | `rl_controller` | 手臂強化學習環境：以 Supervisor 讀取末端與目標球（`TARGET_A/B/C`）世界座標，提供 observation / reward / action 介面，目標點 A→B→C 循環；目前 action 為隨機擾動，留給 RL 模型接入 |

## 需求

- [Webots R2025a](https://cyberbotics.com/)
- Python 3 與 `numpy`（`hw5` 用到）

## 備註

- `.*.wbproj` 為 Webots 自動產生的個人化 GUI 狀態檔，已在 `.gitignore` 排除。
- `worlds/.*.jpg` 是 world 的預覽縮圖，保留在版控中。
