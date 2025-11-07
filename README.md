Демонстрация: черепаха из `turtlesim` едет по карте линий из пакета `line_follower`.

1) Запустить turtlesim (в первом окне)
   set "QT=C:\Qt\5.15.2\msvc2019_64"
set "PATH=%QT%\bin;%PATH%"
set "QT_QPA_PLATFORM_PLUGIN_PATH=%QT%\plugins\platforms"

%ROS2CLI% run turtlesim turtlesim_node

2) Второе окно
   cd /d C:\ros2_ws
   call install\local_setup.bat
   %ROS2CLI% launch line_follower line_follower.launch.py

Сборка пакета после правок
cd /d C:\ros2_ws
colcon build --merge-install
call install\local_setup.bat
