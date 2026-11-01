# 最小 cv2 桩：只为离线冒烟测试，让全局规划器能 import
import numpy as np

COLOR_BGR2GRAY = 6
INTER_NEAREST = 0


def cvtColor(img, code):
    a = np.asarray(img)
    if code == COLOR_BGR2GRAY and a.ndim == 3:
        return (0.114 * a[..., 0] + 0.587 * a[..., 1] + 0.299 * a[..., 2]).astype(np.uint8)
    return a


def resize(img, dsize, interpolation=0):
    a = np.asarray(img)
    h, w = a.shape[:2]
    tw, th = int(dsize[0]), int(dsize[1])
    yi = (np.arange(th) * h // th).clip(0, h - 1)
    xi = (np.arange(tw) * w // tw).clip(0, w - 1)
    return a[yi][:, xi]
