#!/bin/bash

# 从标准输入读取数据
while read -r line; do
  # 获取最后一列数字
  last_column=$(echo "$line" | awk '{print $NF}')

  # 检查最后一列是否为浮点数
  if [[ "$last_column" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    total=$(awk "BEGIN {print $total + $last_column}")
    count=$((count + 1))

    # 更新最大值和最小值
    if [ -z "$min_value" ] || (( $(awk "BEGIN {print ($last_column < $min_value) ? 1 : 0}") )); then
      min_value=$last_column
    fi

    if [ -z "$max_value" ] || (( $(awk "BEGIN {print ($last_column > $max_value) ? 1 : 0}") )); then
      max_value=$last_column
    fi
  else
    echo "Warning: Skipping invalid entry - $line"
  fi
done

# 计算平均值
if [ "$count" -gt 0 ]; then
  average=$(awk "BEGIN {print $total / $count}")
  echo "Average: $average, Minimum: $min_value, Maximum: $max_value"
else
  echo "No valid numbers found in the last column."
fi
