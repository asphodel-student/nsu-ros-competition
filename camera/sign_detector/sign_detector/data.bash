#!/bin/bash
# Поиск папки с именем "ros2_ws"
path=$(find / -name "ros2_ws" -type d -print -quit 2>/dev/null)
# Проверка, была ли найдена папка "ros2_ws"
if [ -z "$path" ]; then
  echo "Папка 'ros2_ws' не найдена"
  exit 1
fi
# Установка переменной окружения "DATA_PATH"
export DATA_PATH="$path"
echo "Путь до папки 'ros2_ws' сохранен в переменной окружения 'DATA_PATH'"