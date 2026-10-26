Djenty Donkey —— 高增益吉他箱头插件（macOS，Apple 芯片 + Intel 通用版，macOS 10.13 及以上）
==========================================================================================

面板上每个旋钮的说明见同目录的 “使用说明.txt” 第三节（那份文件的第一、二节是 Windows 的安装步骤，Mac 请看本文件）。

一、安装（二选一）
------------------
A. 安装包（推荐）
   1. 双击 DjentyDonkey-x.y.z.pkg
   2. 如果提示“无法打开，因为它来自身份不明的开发者”：
      在 Finder 里对着 .pkg 点右键 →“打开”→ 再点“打开”
      （macOS 13 以后也可以：系统设置 → 隐私与安全性 → 下方点“仍要打开”）
      这个程序没有购买苹果开发者证书，所以 macOS 会提示，属于正常现象
   3. 一路继续。默认安装全部三项（“自定”里可以取消不需要的）：
      - 独立程序   /Applications/Djenty Donkey.app
      - VST3 插件  /Library/Audio/Plug-Ins/VST3/        （Cubase、Reaper、Ableton Live、Studio One 等）
      - AU 插件    /Library/Audio/Plug-Ins/Components/  （Logic Pro、GarageBand 等）

B. 免安装版
   打开 Portable 文件夹：
   - Djenty Donkey.app 拖到“应用程序”
   - Djenty Donkey.vst3 复制到 ~/Library/Audio/Plug-Ins/VST3/
   - Djenty Donkey.component 复制到 ~/Library/Audio/Plug-Ins/Components/
   从网上下载的文件会被 macOS 隔离，复制后在“终端”里执行一次（路径按实际位置改）：
     xattr -dr com.apple.quarantine "/Applications/Djenty Donkey.app"
     xattr -dr com.apple.quarantine ~/Library/Audio/Plug-Ins/VST3/"Djenty Donkey.vst3"
     xattr -dr com.apple.quarantine ~/Library/Audio/Plug-Ins/Components/"Djenty Donkey.component"

二、第一次使用（独立程序）
--------------------------
1. 第一次打开会询问“想访问麦克风”：点“允许”（否则听不到吉他）
   点错了：系统设置 → 隐私与安全性 → 麦克风 → 打开 Djenty Donkey
2. 左上角 Options → Audio/MIDI Settings
3. Output / Input 选你的声卡，勾选吉他所在的输入通道
4. 缓冲（Audio buffer size）建议 64 或 128

三、在 DAW 里找不到插件
------------------------
- Logic Pro：设置 → Plug-in 管理器 → 找到 Djenty Donkey → “重置并重新扫描所选项”
  插件在 Audio Units → Djenty Donkey 下
- 其他 DAW：在插件管理里重新扫描 VST3

四、预设文件夹
--------------
文稿/Djenty Donkey/Presets（.xml 文件，可以直接发给朋友，Windows 和 Mac 通用）
