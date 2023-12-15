#!/bin/bash
# Запуск первой команды в новом терминале
gnome-terminal --command="ros2 launch robot_bringup autorace_2023.launch.py"
# Ожидание завершения первой команды
# Запуск второй команды в новом терминале
gnome-terminal --command="ros2 launch controller_node launch_me.launch.py"
# Ожидание завершения второй команды
# Запуск третьей команды в новом терминале
gnome-terminal --command="ros2 run referee_console mission_autorace_2023_referee"
