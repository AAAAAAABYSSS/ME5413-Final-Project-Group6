#!/usr/bin/env python3
import os
import shutil

target_dir = './src/me5413_perception/models/numbers/'
os.makedirs(target_dir, exist_ok=True)

for i in range(1, 10):
    folder_name = f'number{i}'
    src_img = f'./src/me5413_world/models/{folder_name}/materials/textures/{folder_name}.png'
    dst_img = os.path.join(target_dir, f'{i:03}.png')  # 001.png ~ 009.png

    if os.path.exists(src_img):
        shutil.copy(src_img, dst_img)
        print(f'Copied {src_img} -> {dst_img}')
    else:
        print(f'{src_img} not found')

