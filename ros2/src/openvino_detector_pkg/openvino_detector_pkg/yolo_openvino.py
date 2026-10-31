"""yolo_openvino.py - YOLOv5 detection with OpenVINO, without ROS (paper, Sec. V-C).

The model is a YOLOv5 ONNX export (export.py --include onnx, fixed input, e.g. 416 x 416) or its
OpenVINO IR (.xml). Its output (1, N, 5 + nc) already holds, per candidate, the box centre and size
in input pixels, the objectness and the class probabilities (the Detect layer of the export applies
the sigmoid and the grid). Steps:
  letterbox to the input size (aspect kept, pad 114, centred), BGR -> RGB, / 255, NCHW float32;
  inference (OpenVINO, device GPU = the integrated GPU, or CPU);
  confidence = objectness * class probability, a threshold per class, NMS per class (IoU 0.45),
  the boxes mapped back to the source image, at most 100, by confidence.
The class names come from the 'names' metadata of the ONNX file (written by the YOLOv5 export), from
the class_names argument, or are 'class_<i>'.
"""
import ast
import os
import time

import numpy as np

PAD_VALUE = 114
NMS_IOU = 0.45
KEEP_TOP_K = 100


def onnx_names(path):
    """the class names in the metadata of a YOLOv5 ONNX export, or None"""
    if not path.lower().endswith('.onnx'):
        return None
    try:
        import onnx
        m = onnx.load(path, load_external_data=False)
        for p in m.metadata_props:
            if p.key == 'names':
                d = ast.literal_eval(p.value)
                return [d[k] for k in sorted(d)] if isinstance(d, dict) else list(d)
    except Exception:          # noqa: BLE001 - no onnx package or no metadata: the caller decides
        return None
    return None


def letterbox(bgr, size_hw, pad=PAD_VALUE):
    """resize with the aspect kept and pad to size_hw; returns (image, ratio, pad_x, pad_y)"""
    import cv2
    h0, w0 = bgr.shape[:2]
    H, W = size_hw
    r = min(H / float(h0), W / float(w0))
    nw, nh = int(round(w0 * r)), int(round(h0 * r))
    dw, dh = (W - nw) / 2.0, (H - nh) / 2.0
    img = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR) if (nw, nh) != (w0, h0) else bgr
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(pad, pad, pad))
    return img, r, left, top


def nms(boxes, scores, iou):
    """indices kept by greedy NMS; boxes (n, 4) as x1, y1, x2, y2"""
    order = np.argsort(-scores)
    keep = []
    area = (boxes[:, 2] - boxes[:, 0]).clip(0) * (boxes[:, 3] - boxes[:, 1]).clip(0)
    while order.size:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = (xx2 - xx1).clip(0) * (yy2 - yy1).clip(0)
        ov = inter / (area[i] + area[rest] - inter + 1e-9)
        order = rest[ov <= iou]
    return keep


def decode(out, ratio, pad_x, pad_y, src_w, src_h, names, thresholds, nms_iou=NMS_IOU, top_k=KEEP_TOP_K):
    """detections (dicts in source pixels) from the output (1, N, 5 + nc) of one image"""
    p = out.reshape(-1, out.shape[-1]).astype(np.float32, copy=False)   # an FP16 export gives float16
    nc = p.shape[1] - 5
    thr = np.asarray(thresholds, dtype=np.float32)
    obj = p[:, 4]
    # a candidate can pass only if its objectness reaches the smallest threshold (conf <= obj)
    m = obj >= thr.min()
    p = p[m]
    if not len(p):
        return []
    conf = p[:, 5:] * p[:, 4:5]
    xy, wh = p[:, 0:2], p[:, 2:4]
    boxes = np.concatenate([xy - wh / 2.0, xy + wh / 2.0], axis=1)
    dets = []
    for c in range(nc):
        s = conf[:, c]
        mc = s >= thr[c]
        if not np.any(mc):
            continue
        b, sc = boxes[mc], s[mc]
        for k in nms(b, sc, nms_iou):
            x1 = min(max((b[k, 0] - pad_x) / ratio, 0.0), float(src_w))
            y1 = min(max((b[k, 1] - pad_y) / ratio, 0.0), float(src_h))
            x2 = min(max((b[k, 2] - pad_x) / ratio, 0.0), float(src_w))
            y2 = min(max((b[k, 3] - pad_y) / ratio, 0.0), float(src_h))
            w, h = x2 - x1, y2 - y1
            if w <= 1.0 or h <= 1.0:
                continue
            dets.append({'class_id': c, 'class_name': names[c] if c < len(names) else 'class_%d' % c,
                         'confidence': float(sc[k]), 'center_x': float(x1 + w / 2.0),
                         'center_y': float(y1 + h / 2.0), 'width': float(w), 'height': float(h)})
    dets.sort(key=lambda d: d['confidence'], reverse=True)
    return dets[:top_k]


class YoloOpenVino:
    """one compiled model and one inference request; detect() is not reentrant"""

    def __init__(self, model_path, device='GPU', class_names=None, class_thresholds=None,
                 default_threshold=0.5, nms_iou=NMS_IOU, log=print):
        import openvino as ov
        if not os.path.exists(model_path):
            raise FileNotFoundError(model_path)
        self.core = ov.Core()
        model = self.core.read_model(model_path)
        shape = model.input(0).get_partial_shape()
        if shape.is_dynamic:
            raise ValueError('the input of %s is dynamic; export with a fixed size (--imgsz 416)' % model_path)
        n, c, h, w = [int(d) for d in model.input(0).get_shape()]
        self.size_hw = (h, w)
        nout = int(model.output(0).get_shape()[-1])
        self.names = list(class_names) if class_names else (onnx_names(model_path) or
                                                             ['class_%d' % i for i in range(nout - 5)])
        if len(self.names) != nout - 5:
            raise ValueError('%d class names for %d classes of the model' % (len(self.names), nout - 5))
        th = dict(class_thresholds or {})
        self.thresholds = [float(th.get(nm, default_threshold)) for nm in self.names]
        self.nms_iou = nms_iou
        self.device = device
        cfg = {'PERFORMANCE_HINT': 'LATENCY'}
        try:
            self.compiled = self.core.compile_model(model, device, cfg)
        except Exception as ex:      # noqa: BLE001 - e.g. no GPU driver: run on the CPU and say so
            log('openvino: cannot compile for %s (%s); using CPU' % (device, ex))
            self.device = 'CPU'
            self.compiled = self.core.compile_model(model, 'CPU', cfg)
        self.request = self.compiled.create_infer_request()
        self.blob = np.empty((1, 3, h, w), dtype=np.float32)
        shown = ', '.join('%s %.2f' % (nm, t) for nm, t in zip(self.names, self.thresholds)) \
            if len(self.names) <= 10 else '%s ... (thresholds %.2f..%.2f)' % (', '.join(self.names[:5]),
                                                                           min(self.thresholds), max(self.thresholds))
        log('openvino %s: %s on %s, input %dx%d, %d classes: %s' % (
            ov.__version__, os.path.basename(model_path), self.device, w, h, len(self.names), shown))

    def detect(self, bgr):
        """(detections, {'pre_ms', 'infer_ms', 'post_ms'})"""
        t0 = time.perf_counter()
        img, r, px, py = letterbox(bgr, self.size_hw)
        # BGR -> RGB, HWC -> CHW, / 255 into the preallocated blob
        np.multiply(img[:, :, ::-1].transpose(2, 0, 1), 1.0 / 255.0, out=self.blob[0], casting='unsafe')
        t1 = time.perf_counter()
        self.request.infer({0: self.blob})
        out = self.request.get_output_tensor(0).data
        t2 = time.perf_counter()
        dets = decode(out, r, px, py, bgr.shape[1], bgr.shape[0], self.names, self.thresholds, self.nms_iou)
        t3 = time.perf_counter()
        return dets, {'pre_ms': 1e3 * (t1 - t0), 'infer_ms': 1e3 * (t2 - t1), 'post_ms': 1e3 * (t3 - t2)}
