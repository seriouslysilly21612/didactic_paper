"""Tests of the parts of yolo_openvino.py that need no model: letterbox, decode, NMS."""
import numpy as np

from openvino_detector_pkg.yolo_openvino import decode, letterbox, nms


def test_letterbox_848x480_to_416():
    img = np.zeros((480, 848, 3), dtype=np.uint8)
    out, r, px, py = letterbox(img, (416, 416))
    assert out.shape == (416, 416, 3)
    assert abs(r - 416 / 848.0) < 1e-9
    assert px == 0 and py in (90, 91)
    assert (out[0, 0] == 114).all() and (out[py + 1, 1] == 0).all()


def test_nms_keeps_the_best_of_overlapping_boxes():
    boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11], [50, 50, 60, 60]], dtype=np.float32)
    scores = np.array([0.6, 0.9, 0.7], dtype=np.float32)
    assert sorted(nms(boxes, scores, 0.45)) == [1, 2]


def test_decode_maps_back_to_the_source_image():
    names = ['tennis_ball', 'apple']
    r, px, py = 0.5, 0.0, 88.0               # a 832 x 480 image letterboxed to 416 x 416
    out = np.zeros((1, 4, 7), dtype=np.float32)
    # cx, cy, w, h (input pixels), objectness, p(tennis_ball), p(apple)
    out[0, 0] = [100, 188, 20, 20, 0.9, 0.95, 0.01]    # tennis ball, conf 0.855
    out[0, 1] = [101, 189, 20, 20, 0.8, 0.90, 0.01]    # the same ball, suppressed by NMS
    out[0, 2] = [300, 200, 40, 40, 0.7, 0.10, 0.80]    # apple, conf 0.56
    out[0, 3] = [200, 200, 40, 40, 0.3, 0.90, 0.00]    # below the threshold
    d = decode(out, r, px, py, 832, 480, names, [0.5, 0.5])
    assert [x['class_name'] for x in d] == ['tennis_ball', 'apple']
    ball = d[0]
    assert abs(ball['confidence'] - 0.855) < 1e-6
    assert abs(ball['center_x'] - 200.0) < 1e-4 and abs(ball['center_y'] - 200.0) < 1e-4
    assert abs(ball['width'] - 40.0) < 1e-4 and abs(ball['height'] - 40.0) < 1e-4


def test_decode_float16_output():
    out = np.zeros((1, 1, 6), dtype=np.float16)
    out[0, 0] = [208, 208, 400, 400, 0.9, 0.9]          # w * h overflows float16
    d = decode(out, 1.0, 0.0, 0.0, 416, 416, ['a'], [0.5])
    assert len(d) == 1 and d[0]['width'] > 300
