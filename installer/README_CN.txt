Djenty Donkey 0.3.0 —— 高增益吉他箱头插件（Windows 64 位）
==========================================================

一、安装（二选一）
------------------
A. 安装版（推荐）
   1. 双击 DjentyDonkey-Setup-0.3.0.exe
   2. 如果弹出“Windows 已保护你的电脑”：点“更多信息”→“仍要运行”
      （这个程序没有购买数字签名证书，所以 Windows 会提示，属于正常现象）
   3. 一路下一步。默认会：
      - 在桌面创建 Djenty Donkey 快捷方式
      - 把 VST3 插件装到 C:\Program Files\Common Files\VST3\（DAW 会自动扫描到）

B. 免安装版
   打开 Portable 文件夹，直接双击 “Djenty Donkey.exe” 就能用。
   想在 DAW 里用插件：把 “Djenty Donkey.vst3” 整个文件夹复制到
   C:\Program Files\Common Files\VST3\

二、第一次使用（独立程序）
--------------------------
1. 左上角 Options → Audio/MIDI Settings
2. Audio device type 选 ASIO（声卡有 ASIO 驱动时，延迟最低）
   没有 ASIO 的话选 Windows Audio
3. Device 选你的声卡，勾选吉他所在的输入通道
4. 缓冲（Buffer）建议 64 或 128

三、面板说明
------------
INPUT        输入电平 + 电平表
             调法：用力扫弦，让电平表的峰值落在绿色区域（-18 ~ -6 dB）
             超过绿色区域：调低 Input；够不到：调高 Input
             顶部红灯亮 = 输入过载（点一下电平表可以复位）
预设栏       右上角：下拉选择预设，< > 切换，SAVE 保存，DELETE 删除自己的预设
             FOLDER 打开预设文件夹（文档\Djenty Donkey\Presets，.xml 文件可以直接发给朋友）
INPUT GATE   输入门（默认关）：放在箱头前，让闷音 chug 之间完全安静
             Release = 完全关闭需要的时间，10~20ms 最'断'
3-BAND GATE  3 段噪声门：Thresh 关门阈值，Release 释放时间
             Low / Mid / High = 各频段降噪深度（高频关得早、低频保留延音）
             Bands 指示每段当前的开合状态
TRANSPOSE    移调 ±12 半音（降调弹奏用）
             LIVE：低延迟（约 6~15ms），现场弹奏用
             HQ：和弦最干净，约 43ms 延迟（DAW 会自动补偿），录音用
COMPRESSOR   压缩
DRIVE        前级过载踏板，Model 按钮二选一：
             PRECISION：基于 Precision Drive 电路，更紧、增益更高，Attack 选低频松紧
               Attack 和真机一样 6 档，数字越大越紧（1 = 470n 最饱满，6 = 33n 最紧）
               经典用法：Drive 0、Volume 8~10、Attack 4~5 → 收紧低频
               （v0.5 起 Attack 按真机，旧工程和预设会自动换算到同一个电容；
                 旧的 82n、220n 两档真机没有，换到最接近的 68n、330n）
             TS9：基于 Ibanez TS9 Tube Screamer 电路，中频凸起、更圆润（没有 Attack）
               经典用法：Drive 0~2、Level 8~10、Tone 5~6
             两个都按原机电路逐个元件模拟，切换时无爆音
DONKEY HEAD  箱头：CLEAN / BLUE / RED 三通道（5150 / 6505 元件级模拟，含功放）
             Gain / Master 的刻度按真实 6505 校准过（v0.4），和真机旋钮读数一致
             Master 推功放：3 以下功放基本干净，3~5 逐渐压缩，7 以上完全饱和
             （v0.3 的工程和预设打开时会自动换算，声音不变）
             Volume 在功放之后只调音量不改音色（相当于衰减器）：
             Master 推到喜欢的饱和度，再用 Volume 把音量降下来
             Presence / Resonance 是功放负反馈电路，调高频 / 低频的'松紧'
CABINET IR   点 LOAD IR 选 .wav 箱体 IR，或者把文件直接拖进去
             < > 切换同文件夹的上一个/下一个 IR，BUILT-IN 回到内置箱体
10-BAND EQ   十段均衡
PHASER / DELAY / REVERB  后级效果
OUTPUT       总音量

四、注意
--------
- 第一次打开时音量可能比较大，先把 OUTPUT 调低
- 加载的 IR 文件如果被移动或删除，下次打开会回到内置箱体
- 卸载：开始菜单 → Djenty Donkey → Uninstall
