"""Save native 4K MJPEG frames independently of the GUI preview."""
import csv
from datetime import datetime
import io
import json
from pathlib import Path
import tempfile
import threading
import time

import cv2
from PIL import Image


class CameraCapture(threading.Thread):
    def __init__(self, camera, output_root, metadata):
        super().__init__(daemon=True)
        self.camera = camera
        self.output_root = Path(output_root)
        self.metadata = dict(metadata)
        self.stop_event = threading.Event()
        self.directory = None
        self.latest = None
        self.frame_count = 0
        self.first_ns = self.last_ns = None
        self.max_interval_ns = 0
        self.long_intervals = 0
        self.error = None

    @property
    def measured_fps(self):
        if self.frame_count < 2 or self.last_ns == self.first_ns:
            return 0.0
        return (self.frame_count - 1) * 1e9 / (self.last_ns - self.first_ns)

    def run(self):
        try:
            readback = dict(width=int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
                            height=int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                            fps=self.camera.get(cv2.CAP_PROP_FPS),
                            fourcc=int(self.camera.get(cv2.CAP_PROP_FOURCC)),
                            auto_exposure=self.camera.get(cv2.CAP_PROP_AUTO_EXPOSURE),
                            exposure=self.camera.get(cv2.CAP_PROP_EXPOSURE),
                            gain=self.camera.get(cv2.CAP_PROP_GAIN),
                            autofocus=self.camera.get(cv2.CAP_PROP_AUTOFOCUS),
                            focus=self.camera.get(cv2.CAP_PROP_FOCUS))
            if ((readback['width'], readback['height']) != (3840, 2160)
                    or abs(readback['fps'] - 30) > 0.1
                    or readback['fourcc'] != cv2.VideoWriter_fourcc(*'MJPG')):
                raise RuntimeError(f'相机不支持本次 4K/MJPG/30 fps 请求：{readback}')
            if not self.camera.set(cv2.CAP_PROP_CONVERT_RGB, 0):
                raise RuntimeError('当前相机后端不支持原始 MJPEG 采集')
            self.output_root.mkdir(parents=True, exist_ok=True)
            self.directory = Path(tempfile.mkdtemp(
                prefix=datetime.now().strftime('%Y%m%d_%H%M%S_'), dir=self.output_root))
            frames = self.directory / 'frames'
            frames.mkdir()
            self.metadata.update(status='recording', requested=dict(width=3840, height=2160, fps=30),
                                 readback=readback, started_at=datetime.now().astimezone().isoformat(),
                                 image_format='native_camera_jpeg_without_undistortion',
                                 timestamp_basis='host_after_read; not exposure time',
                                 long_interval_threshold_ms=50)
            self._save_metadata()
            with (self.directory / 'timestamps.csv').open('x', newline='', buffering=1) as stream:
                writer = csv.writer(stream)
                writer.writerow(['frame_index', 'filename', 'host_monotonic_ns', 'host_unix_ns', 'bytes'])
                while not self.stop_event.is_set():
                    ok, packet = self.camera.read()
                    now = time.monotonic_ns()
                    wall = time.time_ns()
                    if not ok or packet is None:
                        raise RuntimeError('相机读帧失败，采集已停止，已有数据保留')
                    if packet.ndim > 2:
                        raise RuntimeError('相机返回了解码图像，未得到原始 MJPEG 数据')
                    payload = packet.tobytes()
                    if not payload.startswith(b'\xff\xd8') or b'\xff\xd9' not in payload:
                        raise RuntimeError('相机返回不完整的 JPEG 帧')
                    with Image.open(io.BytesIO(payload)) as image:
                        if image.size != (3840, 2160):
                            raise RuntimeError(f'实际 JPEG 尺寸变化为 {image.size}，停止采集')
                    name = f'{self.frame_count:08d}.jpg'
                    partial = frames / (name + '.partial')
                    with partial.open('xb') as output:
                        output.write(payload)
                    partial.rename(frames / name)
                    writer.writerow([self.frame_count, f'frames/{name}', now, wall, len(payload)])
                    if self.first_ns is None:
                        self.first_ns = now
                    else:
                        gap = now - self.last_ns
                        self.max_interval_ns = max(self.max_interval_ns, gap)
                        self.long_intervals += int(gap > 50_000_000)
                    self.last_ns = now
                    self.frame_count += 1
                    self.latest = (self.frame_count, payload)
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.camera.release()
            if self.directory is not None:
                self.metadata.update(status='failed' if self.error else 'complete', error=self.error,
                                     frame_count=self.frame_count, measured_fps=self.measured_fps,
                                     max_interval_ms=self.max_interval_ns / 1e6,
                                     intervals_over_50ms=self.long_intervals,
                                     first_host_monotonic_ns=self.first_ns, last_host_monotonic_ns=self.last_ns,
                                     finished_at=datetime.now().astimezone().isoformat())
                try:
                    self._save_metadata()
                except OSError as exc:
                    self.error = f'{self.error or ""} 元数据保存失败：{exc}'

    def _save_metadata(self):
        temporary = self.directory / 'metadata.json.partial'
        temporary.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(self.directory / 'metadata.json')
