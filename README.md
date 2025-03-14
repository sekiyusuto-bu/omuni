. /opt/ros/humble/setup.bash(一番最初に一回だけ打つ)
. install/setup.bash(新しくターミナルを開くたびに書く)
ros2 run omuni 3rin
ros2 run shoko shoko
ros2 run can_conection can_send
ros2 run joy joy_node
ros2 launch robomas_plugins (robomas_pluginsのノード名は長いのでrobomasまで打ったらTabを押すとノード名が完全になる)
