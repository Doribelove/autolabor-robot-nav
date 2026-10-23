"""NumPy/OpenCV/Pillow versions of the pinned production input transforms.

These avoid a PyTorch dependency on J6M. Verify against the saved calibration
tensors before using them with a new dependency build. No ROS side effects.
"""
import numpy as np


def check_image(image):
    if (not isinstance(image, np.ndarray) or image.dtype != np.uint8
            or image.ndim != 3 or image.shape[2] != 3
            or min(image.shape[:2]) < 1 or max(image.shape[:2]) > 8192):
        raise ValueError('Expected a nonempty BGR uint8 image, at most 8192 per side')


def detector_input(image):
    """Match custom Ultralytics LetterBox(auto=False, scaleup=True, center=True)."""
    import cv2
    check_image(image)
    height, width = image.shape[:2]
    ratio = min(576 / height, 1024 / width)
    resized = (round(width * ratio), round(height * ratio))
    if min(resized) < 1:
        raise ValueError('Image aspect ratio produces an empty resized dimension')
    dw, dh = (1024 - resized[0]) / 2, (576 - resized[1]) / 2
    left, right = round(dw - .1), round(dw + .1)
    top, bottom = round(dh - .1), round(dh + .1)
    frame = cv2.resize(image, resized, interpolation=cv2.INTER_LINEAR) if resized != (width, height) else image
    frame = cv2.copyMakeBorder(frame, top, bottom, left, right,
                               cv2.BORDER_CONSTANT, value=(114, 114, 114))
    tensor = np.ascontiguousarray(frame[:, :, ::-1].transpose(2, 0, 1)[None],
                                  dtype=np.float32) / np.float32(255)
    if tensor.shape != (1, 3, 576, 1024):
        raise ValueError('Unexpected detector input dimensions')
    return tensor, (ratio, left, top, height, width)


def classifier_input(image):
    """Match PIL bilinear short-edge Resize(224), CenterCrop(224), ToTensor.

    Normalization is identity (mean zero/std one). Pillow supplies antialiasing
    for bilinear resizing, as in the pinned torchvision PIL path.
    """
    from PIL import Image
    check_image(image)
    height, width = image.shape[:2]
    frame = Image.fromarray(np.ascontiguousarray(image[:, :, ::-1]))
    if width <= height:
        resized = (224, int(224 * height / width))
    else:
        resized = (int(224 * width / height), 224)
    if resized != (width, height):
        frame = frame.resize(resized, resample=Image.Resampling.BILINEAR)
    left, top = round((resized[0] - 224) / 2), round((resized[1] - 224) / 2)
    frame = frame.crop((left, top, left + 224, top + 224))
    return np.ascontiguousarray(np.asarray(frame).transpose(2, 0, 1)[None],
                                dtype=np.float32) / np.float32(255)
