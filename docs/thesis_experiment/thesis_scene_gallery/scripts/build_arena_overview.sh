#!/usr/bin/env bash
set -euo pipefail

gallery_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image_root="${gallery_root}/arena_images"
font_path="/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

if [[ ! -r "${font_path}" ]]; then
  echo "Required CJK font is unavailable: ${font_path}" >&2
  exit 2
fi

ffmpeg -hide_banner -loglevel error -y \
  -i "${image_root}/01_residential.png" \
  -i "${image_root}/02_bookstore.png" \
  -i "${image_root}/03_hospital.png" \
  -i "${image_root}/04_factory.png" \
  -i "${image_root}/05_small_warehouse.png" \
  -filter_complex "[0:v]scale=1200:700:force_original_aspect_ratio=decrease,pad=1200:760:(ow-iw)/2:0:white,drawtext=fontfile=${font_path}:text='(a) 住宅环境':fontcolor=#20252B:fontsize=38:x=(w-text_w)/2:y=710[a];[1:v]scale=1200:700:force_original_aspect_ratio=decrease,pad=1200:760:(ow-iw)/2:0:white,drawtext=fontfile=${font_path}:text='(b) 书店环境':fontcolor=#20252B:fontsize=38:x=(w-text_w)/2:y=710[b];[2:v]scale=1200:700:force_original_aspect_ratio=decrease,pad=1200:760:(ow-iw)/2:0:white,drawtext=fontfile=${font_path}:text='(c) 医院环境':fontcolor=#20252B:fontsize=38:x=(w-text_w)/2:y=710[c];[3:v]scale=1200:700:force_original_aspect_ratio=decrease,pad=1200:760:(ow-iw)/2:0:white,drawtext=fontfile=${font_path}:text='(d) 工厂环境':fontcolor=#20252B:fontsize=38:x=(w-text_w)/2:y=710[d];[4:v]scale=1200:700:force_original_aspect_ratio=decrease,pad=1200:760:(ow-iw)/2:0:white,drawtext=fontfile=${font_path}:text='(e) 小型仓库':fontcolor=#20252B:fontsize=38:x=(w-text_w)/2:y=710[e];color=c=white:s=3600x1520[bg];[bg][a]overlay=0:0[t1];[t1][b]overlay=1200:0[t2];[t2][c]overlay=2400:0[t3];[t3][d]overlay=600:760[t4];[t4][e]overlay=1800:760[out]" \
  -map "[out]" -frames:v 1 \
  "${image_root}/00_arena_environment_overview.png"

echo "built 00_arena_environment_overview.png"
