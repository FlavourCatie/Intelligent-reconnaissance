import matplotlib.pyplot as plt
from PIL import Image

# 读取图像
img_path = 'map.jpg'  # 替换成你的图片路径
img = Image.open(img_path)

# 显示图像
fig, ax = plt.subplots()
ax.imshow(img)

# 定义点击事件的回调函数
def on_click(event):
    # 获取点击位置的坐标
    x, y = int(event.xdata), int(event.ydata)
    print(f"点击的像素坐标: ({x}, {y})")

    world_x = x + (-2094.5277099609375)
    world_y = y + (-2637.31982421875)
    print(f"点击的世界坐标: ({world_x}, {world_y})")


    # 获取该坐标的像素值
    pixel_value = img.getpixel((x, y))
    print(f"该像素点的RGB值: {pixel_value}")

# 设置点击事件监听
fig.canvas.mpl_connect('button_press_event', on_click)

# 展示图像，等待点击
plt.show()
