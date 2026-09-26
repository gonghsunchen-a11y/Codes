# RoboCupJunior 足球機器人｜核心控制程式閱讀索引

本索引作為備審報告「附件一：核心控制程式」的補充，說明兩塊 Teensy 4.1 的主要程式、共用控制函式及全向鏡相機程式。點擊行號可直接查看對應程式碼。

## 四個主要檔案

| 檔案 | 分工 |
|---|---|
| [main_ball.cpp](TEENSY/src/main_ball.cpp) | Sensor Integration and Offensive Decision-making (Teensy 4.1 #1) |
| [sub_ball.cpp](TEENSY/src/sub_ball.cpp) | Line Sensing and Chassis Control (Teensy 4.1 #2) |
| [Robot.h](TEENSY/include/Robot.h) | Shared Data, Sensor Parsing and Motion Control |
| [Omnidirectional_cam](cam.codes/Omnidirectional_cam) | Omnidirectional Vision, Goal Localization and Ball Detection |

## 版本與閱讀方式

- 分支：`BC`
- 索引版本：`29c50cbf982313a64b01fe390166f67555b7d28b`
- 以下採固定提交連結，後續修改程式不會使這份索引連到不同內容。
- `Robot.h` 不只有宣告，也包含感測接收與馬達、航向、擊球控制的實作。
- 這是程式閱讀索引，不代表已完成編譯或實機驗證；保留的測試區塊與目前控制方式，詳見末節。

## main_ball.cpp：感測整合與進攻決策

| 行號 | Function / Block | 功能說明 |
|---|---|---|
| [19–27](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L19-L27) | Edge Braking Parameters | 設定左右、前後開始減速與停止的位置。 |
| [87–173](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L87-L173) | Ultrasonic Ranging | 回波中斷、依序觸發、距離換算、平滑與初始化。 |
| [519–545](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L519-L545) | applyIRChase() | 依紅外線足球方向產生 Vx、Vy。 |
| [547–675](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L547-L675) | applyOmniEdgeBrake() | 依定位座標調整朝邊界的速度；目前左右分支會提前 return。 |
| [677–704](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L677-L704) | sendMovePacket() | 將 Vx、Vy、射門偏角與檢查碼經 UART 傳送。 |
| [706–830](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L706-L830) | updateKalmanPosition() | 超音波座標換算、定位初始化、不確定程度與依序融合。 |
| [731–743](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L731-L743) | Ultrasonic Position Calculation | 利用左右、前後距離差，換算 X、Y 座標。 |
| [775–829](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L775-L829) | Sequential Sensor Fusion | 單軸卡爾曼更新、超音波修正、相機修正與結果保存。 |
| [832–849](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L832-L849) | setup() | 控制板、超音波、持球感測與 ESC 初始化。 |
| [851–956](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L851-L956) | Main State Machine | 待機、光感校正與進攻狀態切換。 |
| [958–973](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L958-L973) | Sensor Update and Ball Possession | 讀取 IR、超音波、相機與持球訊號，更新融合座標。 |
| [981–1026](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L981-L1026) | Ball Possession and Shooting | 依 X 座標選擇偏角、限制射門區域、50 ms 計時與單次觸發。 |
| [1056–1084](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L1056-L1084) | Ball Tracking Source Selection | 檢查 IR 與相機資料，使用相機提供的足球角度與距離。 |
| [1086–1127](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L1086-L1127) | Close-range Ball Orbiting | 設定速度、正前方判斷、繞球側別與指數偏角。 |
| [1129–1155](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L1129-L1155) | Velocity Command Calculation | 角度正規化、sin/cos 換算 Vx、Vy，近距離向前推進。 |
| [1162–1182](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L1162-L1182) | No-ball Behavior | 定位有效時返回中心；定位無效時停止。 |
| [1186–1196](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/main_ball.cpp#L1186-L1196) | Motion Command Output | 套用座標減速並傳送移動封包。 |

## sub_ball.cpp：白線感測與底盤控制

| 行號 | Function / Block | 功能說明 |
|---|---|---|
| [23–37](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/sub_ball.cpp#L23-L37) | Line Sensor Pins and Data | M1、M2、多工器選擇腳、32 組光感與校正陣列。 |
| [56–88](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/sub_ball.cpp#L56-L88) | readMux() | 選擇通道、等待 20 μs，再讀取指定多工器。 |
| [90–172](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/sub_ball.cpp#L90-L172) | line_calibrate() | 依序讀取 32 組光感，記錄最大最小值、計算門檻並存入 EEPROM。 |
| [174–223](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/sub_ball.cpp#L174-L223) | fast_update_line_sensor() | 同步切換 16 個通道，分別預讀與讀取 M1/M2，以門檻更新白線位元狀態。 |
| [225–329](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/sub_ball.cpp#L225-L329) | moveBackInBounds() | 光感方向向量合成、記錄首次碰線角度並決定回場方向。 |
| [331–374](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/sub_ball.cpp#L331-L374) | readCommand() | 接收校正、進攻及停止命令。 |
| [376–446](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/sub_ball.cpp#L376-L446) | readMainCore() | 解析主控制器的 7 位元組移動封包與檢查碼。 |
| [448–466](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/sub_ball.cpp#L448-L466) | setup() | 設定通訊與光感腳位，讀取 EEPROM 校正門檻。 |
| [468–503](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/sub_ball.cpp#L468-L503) | Main Loop and Tilt Protection | 讀取航向、移動封包與白線；俯仰角超過門檻時停止。 |
| [505–538](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/src/sub_ball.cpp#L505-L538) | Boundary Priority and Heading Control | 碰線時執行回場；否則設定射門航向及比例係數，呼叫運動控制。 |

## Robot.h：共用資料與控制函式

| 行號 | Function / Block | 功能說明 |
|---|---|---|
| [8–73](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/include/Robot.h#L8-L73) | Parameters and Pins | 角度常數、按鈕、馬達 DIR/PWM、充電與擊球腳位。 |
| [79–105](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/include/Robot.h#L79-L105) | Data Structures and Control Parameters | 陀螺儀、IR、相機資料、32 組光感角度及航向參數。 |
| [137–185](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/include/Robot.h#L137-L185) | Robot_Init() | 初始化串列埠、控制腳位、OLED 與擊球控制。 |
| [188–235](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/include/Robot.h#L188-L235) | readBNO085Yaw() | 19 位元組封包、起始標記、檢查碼、Yaw/Pitch 原始值與度數換算。 |
| [238–263](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/include/Robot.h#L238-L263) | readMaix() | 接收全向相機座標、足球方向及距離資料。 |
| [355–373](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/include/Robot.h#L355-L373) | ballsensor() | 接收 ESP32 傳回的紅外線足球偵測資料。 |
| [376–439](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/include/Robot.h#L376-L439) | SetMotorSpeed() | 將馬達輸出正負轉成 DIR，輸出大小轉成 PWM。 |
| [441–446](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/include/Robot.h#L441-L446) | MotorStop() | 將四顆馬達 PWM 設為零。 |
| [468–512](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/include/Robot.h#L468-L512) | Motor Calibration and RobotIKControl() | 四輪逆運動學換算、輸出校正與可選的漸變控制。 |
| [606–691](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/include/Robot.h#L606-L691) | Vector_Motion() | 角度轉換、最短角度誤差、PD 修正與全向底盤輸出；回場流程呼叫此函式。 |
| [719–757](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/include/Robot.h#L719-L757) | FC_Vector_Motion() | 目標航向誤差與 P 控制；接收呼叫端傳入的 heading_kp。 |
| [855–915](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/TEENSY/include/Robot.h#L855-L915) | kicker_control() | 擊球上升緣判斷、500 ms 間隔、15 ms 擊球脈衝與兩次擊球後的充電等待。 |

## Omnidirectional_cam：全向鏡辨識與定位

| 行號 | Function / Block | 功能說明 |
|---|---|---|
| [1–44](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L1-L44) | Camera and Detection Parameters | 設定相機、UART、LAB 色彩門檻、影像中心、辨識範圍、距離公式係數與五次半徑平均。 |
| [55–72](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L55-L72) | pack_int8() / pack_int16() | 將座標及角度轉換為傳輸用位元組；座標限制於有號 8 位元範圍。 |
| [75–84](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L75-L84) | point_radius() / blob_radius() / angle_diff() | 計算影像半徑及兩方向的最短角度差。 |
| [87–144](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L87-L144) | merge_goal_parts() | 篩選球門色塊，以最大色塊為基準，合併半徑與角度接近的區塊。 |
| [187–204](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L187-L204) | calc_goal_position() | 平均最近五次影像半徑，代入距離公式，再利用三角函數計算球門相對座標。 |
| [207–251](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L207-L251) | find_ball_in_circle() | 依橘色色塊、半徑、寬高與面積篩選足球，輸出方向與像素距離。 |
| [254–272](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L254-L272) | Frame Acquisition and Ball Update | 讀取並翻轉影像，每張有效影像更新足球辨識結果。 |
| [274–299](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L274-L299) | Goal Detection and Relative Position | 每 10 張有效影像更新兩側球門；未辨識到某側球門時清除該側半徑歷史。 |
| [301–310](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L301-L310) | Dual-goal Localization | 兩座球門皆可見時，將相對座標取平均並反向，估算機器人場上位置。 |
| [312–330](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L312-L330) | Single-goal Localization | 僅一座球門可見時，利用已知球門位置與進攻方向進行備援定位。 |
| [331–354](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L331-L354) | UART Packet Encoding | 將座標、定位狀態、足球資訊與檢查碼組成 10 位元組封包。 |
| [356–365](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L356-L365) | UART Request Handling | 收到 0xDD 請求後，傳送目前封包。 |
| [368–414](https://github.com/gonghsunchen-a11y/Codes/blob/29c50cbf982313a64b01fe390166f67555b7d28b/cam.codes/Omnidirectional_cam#L368-L414) | Debug Visualization | 啟用 DEBUG_VIEW 時顯示辨識框、範圍圓及定位資訊。 |

## 與報告章節的對照

| 報告內容 | 程式位置 |
|---|---|
| 第 3 章：光感校正與白線讀取 | `sub_ball.cpp`: lines 56–223 |
| 第 4 章：全向鏡球門辨識、距離估算與定位 | `Omnidirectional_cam`: lines 87–144, 187–204, 274–330 |
| 第 5 章：影像足球方向與距離 | `Omnidirectional_cam`: lines 207–251 |
| 第 4 章：超音波座標與卡爾曼融合 | `main_ball.cpp`: lines 706–830 |
| 第 5 章：全向底盤與航向校正 | `Robot.h`: lines 468–512、606–757 |
| 第 5 章：座標減速與白線回場 | `main_ball.cpp`: lines 547–675; `sub_ball.cpp`: lines 225–329 |
| 第 5 章：追球、繞球與射門判斷 | `main_ball.cpp`: lines 981–1182 |
| 第 5 章：擊球電路控制 | `Robot.h`: lines 855–915 |

## 建議閱讀順序

1. 光感校正與白線偵測：`sub_ball.cpp`: lines 56–223。
2. 白線回場：`sub_ball.cpp`: lines 225–329。
3. 全向鏡辨識與定位：`Omnidirectional_cam`，先讀 `calc_goal_position()` 與主循環中的定位區塊。
4. 定位融合：`main_ball.cpp`: lines 706–830。
5. 追球、繞球與射門：`main_ball.cpp`: lines 981–1182。
6. 航向與四輪輸出：`Robot.h`: lines 468–512、606–757。
7. 整體循環：`main_ball.cpp` 與 `sub_ball.cpp` 的 `loop()`。

## 版本閱讀注意事項

- `Omnidirectional_cam` 146–186 行為三引號包住的舊版合併方式，不會執行；足球距離是影像像素半徑，不是公分。球門距離則使用經驗公式換算。

- 校正與一般白線偵測的掃描方式不同：校正依序讀取 32 組；一般偵測同步選擇兩顆多工器的通道，再分別讀取 M1、M2，共切換 16 次。
- `main_ball.cpp`: lines 335–517保留足球預測與向量場函式，但目前主要追球流程改用 1086–1155 行的偏角方式；另有多段註解中的舊版本，不應視為目前正在執行的功能。
- 航向控制有兩個路徑：回場呼叫 `Vector_Motion()`（含 P、D 項），一般移動與射門呼叫 `FC_Vector_Motion()`（P 控制，係數由呼叫端傳入）。
- 此版本的座標減速函式仍在左右分支提前返回；無球且定位有效時會返回中心。閱讀報告的兩軸獨立減速示意或無球停止流程時，須區分設計說明與此版本實作。
- 程式包含測試設定；這份索引僅核對原始碼位置，不代表編譯或實機驗證結果。
