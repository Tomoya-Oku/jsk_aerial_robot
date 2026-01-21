
# Dracomancerとは (What is Dracomancer?)

# ディレクトリ構成 (Directory Structure)
- config/
- scripts/
- include/
    - control/
    - model/
- launch/

    $ roslaunch で起動するもの

- include/
- plugins/
- scripts/

    主にcatkin buildが不要なもの
    
    - feedback_joint.py: 関節フィードバックシステムを起動 (引数: robot_nameはデフォルトでdragon)
    - keyboard_command.py: モータ類をキーボードで操作
    - mocap_test.py: (使用予定なし) モーキャプの利用

- src/

    主にcatkin buildが必要なもの