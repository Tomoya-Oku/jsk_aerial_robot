# gen_random_dots.py
import random
from PIL import Image, ImageDraw

W, H = 400, 600          # テクスチャ解像度
N = 20                    # 点の数
R = 8                      # 点半径(px)
bg = (240, 240, 240)       # 背景
fg = (0, 0, 0)          # 点色

img = Image.new("RGB", (W, H), bg)
draw = ImageDraw.Draw(img)

for _ in range(N):
    x = random.randint(R, W - R - 1)
    y = random.randint(R, H - R - 1)
    draw.ellipse((x - R, y - R, x + R, y + R), fill=fg)

img.save("random_dots.png")
print("saved random_dots.png")
