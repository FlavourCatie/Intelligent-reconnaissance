from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from skimage import color
from skimage.morphology import skeletonize
from scipy.spatial import distance

def skeleton_refinement(image_pil: Image.Image) -> Image.Image:
    # 将PIL图像转换为NumPy数组
    image = np.array(image_pil)

    # 如果图像是RGB图像，转换为灰度图像
    if image.ndim == 3:
        gray_image = color.rgb2gray(image)  # 将RGB图像转换为灰度图像
    else:
        gray_image = image

    # 进行二值化处理
    binary_image = gray_image > 0.5  # 使用0.5作为阈值，生成二值图像

    # 骨架细化
    skeleton = skeletonize(binary_image)

    # 将细化后的骨架图像转换回PIL图像
    # 将布尔型数组转换为0-255的图像
    skeleton_image = (skeleton * 255).astype(np.uint8)

    # 将单通道骨架图像转换为三通道图像
    skeleton_rgb_image = np.stack([skeleton_image] * 3, axis=-1)  # 创建一个三通道图像

    skeleton_pil_image = Image.fromarray(skeleton_rgb_image)


    # 保存为 TGA 格式
    tga_image_path = "skeletonized_image.tga"
    skeleton_pil_image.save(tga_image_path, format='TGA')

    # 保存为 PNG 格式
    png_image_path = "skeletonized_image.png"
    skeleton_pil_image.save(png_image_path, format='PNG')

    # 重新打开保存的 TGA 图像以确保返回类型
    return Image.open(tga_image_path)


def skeleton_point(img_skl: Image.Image, start_x: int, start_y: int, end_x: int, end_y: int):
    # 将 PIL 图像转换为 NumPy 数组
    img_array = np.array(img_skl)

    # 假设骨架点是亮色（例如，白色），可以用阈值来确定有效点
    # 定义阈值
    threshold = 200  # 可调节阈值，值越高越严格

    # 创建二值图像，找到亮色区域（白色或接近白色）
    binary_image = np.all(img_array >= threshold, axis=-1)  # 对每个通道应用阈值

    # 获取所有有效点的坐标（骨架点）
    y_indices, x_indices = np.nonzero(binary_image)

    # 将有效点的坐标组合成一个数组
    valid_points = np.column_stack((x_indices, y_indices))

    # 找到最近的点
    start_point = np.array([start_x, start_y])
    end_point = np.array([end_x, end_y])

    # 计算距离并找到最近的有效点
    start_distances = distance.cdist(valid_points, [start_point])
    end_distances = distance.cdist(valid_points, [end_point])

    # 找到最近的点的索引
    closest_start_index = np.argmin(start_distances)
    closest_end_index = np.argmin(end_distances)

    # 获取最近的点坐标
    closest_start = valid_points[closest_start_index]
    closest_end = valid_points[closest_end_index]

    # 返回新的点坐标
    start_x_skl, start_y_skl = closest_start
    end_x_skl, end_y_skl = closest_end

    return start_x_skl, start_y_skl, end_x_skl, end_y_skl
