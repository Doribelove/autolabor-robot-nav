#!/usr/bin/env bash
set -euo pipefail

# Compose the six paper-ready scene plates into one labelled thesis figure.

gallery_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image_root="${gallery_root}/images"
font_path="/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

if [[ ! -r "${font_path}" ]]; then
  echo "Required CJK font is unavailable: ${font_path}" >&2
  exit 2
fi

ffmpeg -hide_banner -loglevel error -y \
  -i "${image_root}/02_static_avoidance.png" \
  -i "${image_root}/03_single_crossing.png" \
  -i "${image_root}/04_partially_occluded_crossing.png" \
  -i "${image_root}/05_head_on_yield.png" \
  -i "${image_root}/06_sequential_interaction.png" \
  -i "${image_root}/07_width_change_corridor.png" \
  -filter_complex "[0:v]scale=1200:561:flags=lanczos,pad=1200:620:0:0:white,drawtext=fontfile=${font_path}:text='(a) 静态障碍避让':fontcolor=#20252B:fontsize=38:x=(w-text_w)/2:y=570[a];[1:v]scale=1200:561:flags=lanczos,pad=1200:620:0:0:white,drawtext=fontfile=${font_path}:text='(b) 单行人横穿':fontcolor=#20252B:fontsize=38:x=(w-text_w)/2:y=570[b];[2:v]scale=1200:561:flags=lanczos,pad=1200:620:0:0:white,drawtext=fontfile=${font_path}:text='(c) 局部遮挡横穿':fontcolor=#20252B:fontsize=38:x=(w-text_w)/2:y=570[c];[3:v]scale=1200:561:flags=lanczos,pad=1200:620:0:0:white,drawtext=fontfile=${font_path}:text='(d) 迎面交互与让行':fontcolor=#20252B:fontsize=38:x=(w-text_w)/2:y=570[d];[4:v]scale=1200:561:flags=lanczos,pad=1200:620:0:0:white,drawtext=fontfile=${font_path}:text='(e) 连续双行人交互':fontcolor=#20252B:fontsize=38:x=(w-text_w)/2:y=570[e];[5:v]scale=1200:561:flags=lanczos,pad=1200:620:0:0:white,drawtext=fontfile=${font_path}:text='(f) 宽度变化走廊':fontcolor=#20252B:fontsize=38:x=(w-text_w)/2:y=570[f];[a][b][c][d][e][f]xstack=inputs=6:layout=0_0|1200_0|2400_0|0_620|1200_620|2400_620[out]" \
  -map "[out]" -frames:v 1 \
  "${image_root}/00_six_scene_overview.png"

echo "built 00_six_scene_overview.png"
