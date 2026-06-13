#!/usr/bin/env python3
"""
Convert ZED SVO2 -> MCAP (foxglove-sdk, version dùng foxglove.messages).

- /ego/imu, /ego/vio/system_info : JSON schema tự định nghĩa (đúng tên trường yaml)
- các topic còn lại               : schema built-in của foxglove.messages

Quy trình: CẮT svo2 theo start/end (HH:MM:SS) -> lưu file svo2 đã cắt vào
thư mục user chỉ định -> CONVERT file đã cắt sang MCAP.

Cài: pip install foxglove-sdk av numpy   (pyzed có sẵn sau khi cài ZED SDK)
Dùng:
  python svo2_to_mcap_v3.py input.svo2 output.mcap --start 00:00:02 --end 00:00:41 --clip-dir ./clips
  python svo2_to_mcap_v3.py input.svo2 output.mcap --start 00:00:02 --end 00:00:41 --clip-dir ./clips --max-frames 100
"""
import os
import sys
import json
import argparse
import fractions

import numpy as np
import pyzed.sl as sl
import av

import foxglove
from foxglove import Channel
from foxglove.messages import (
    FrameTransform,
    FrameTransforms,
    CompressedVideo,
    CameraCalibration,
    Vector3,
    Quaternion,
    Timestamp,
)

# ---------------------------------------------------------------------------
# JSON schema cho 2 topic không có built-in
# ---------------------------------------------------------------------------
IMU_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": {
            "type": "object",
            "properties": {
                "seconds": {"type": "integer"},
                "nanos": {"type": "integer"},
            },
        },
        "frame_id": {"type": "string"},
        "orientation": {
            "type": "object",
            "properties": {
                "x": {"type": "number"}, "y": {"type": "number"},
                "z": {"type": "number"}, "w": {"type": "number"},
            },
        },
        "angular_velocity": {
            "type": "object",
            "properties": {
                "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"},
            },
        },
        "linear_acceleration": {
            "type": "object",
            "properties": {
                "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"},
            },
        },
    },
}

ROBOTINFO_SCHEMA = {
    "type": "object",
    "properties": {
        "name": {"type": "string"},
        "description": {"type": "string"},
        "world_frame": {"type": "string"},
        "body_frame": {"type": "string"},
        "coordinate_convention": {"type": "string"},
        "serial_number": {"type": "string"},
        "firmware_version": {"type": "string"},
    },
}


def ns_to_ts(ns: int) -> Timestamp:
    return Timestamp(sec=ns // 1_000_000_000, nsec=ns % 1_000_000_000)


def ts_dict(ns: int):
    return {"seconds": ns // 1_000_000_000, "nanos": ns % 1_000_000_000}


class H265Encoder:
    def __init__(self, width, height, fps):
        self.codec = av.CodecContext.create("libx265", "w")
        self.codec.width = width
        self.codec.height = height
        self.codec.pix_fmt = "yuv420p"
        f = int(round(fps)) or 30
        self.codec.framerate = fractions.Fraction(f, 1)
        self.codec.time_base = fractions.Fraction(1, f)
        self.codec.options = {"preset": "fast", "x265-params": "log-level=none"}

    def encode_bgr(self, bgr: np.ndarray) -> bytes:
        frame = av.VideoFrame.from_ndarray(bgr, format="bgr24")
        frame = frame.reformat(format="yuv420p")
        out = b""
        for pkt in self.codec.encode(frame):
            out += bytes(pkt)
        return out

    def flush(self) -> bytes:
        out = b""
        for pkt in self.codec.encode(None):
            out += bytes(pkt)
        return out


def make_calibration(ts, frame_id, w, h, cp):
    fx, fy, cx, cy = cp.fx, cp.fy, cp.cx, cp.cy
    disto = list(cp.disto)
    while len(disto) < 5:
        disto.append(0.0)
    D = [float(x) for x in disto[:5]]
    K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    # Tên field built-in có thể là d/k/r/p (thường) -> chỉnh nếu help() báo khác
    return CameraCalibration(
        timestamp=ts, frame_id=frame_id, width=w, height=h,
        distortion_model="plumb_bob", D=D, K=K, R=R, P=P,
    )


def parse_hhmmss(s: str) -> float:
    """'HH:MM:SS' hoặc 'MM:SS' hoặc 'SS' -> số giây (float)."""
    parts = s.strip().split(":")
    parts = [float(p) for p in parts]
    if len(parts) == 3:
        h, m, sec = parts
    elif len(parts) == 2:
        h, m, sec = 0.0, parts[0], parts[1]
    elif len(parts) == 1:
        h, m, sec = 0.0, 0.0, parts[0]
    else:
        raise ValueError(f"Định dạng thời gian không hợp lệ: {s}")
    return h * 3600 + m * 60 + sec


def clip_svo(input_svo: str, clip_dir: str, start_s: float, end_s: float) -> str:
    """
    Cắt SVO2 từ start_s đến end_s (giây), lưu file mới vào clip_dir.
    Trả về đường dẫn file svo2 đã cắt.
    """
    os.makedirs(clip_dir, exist_ok=True)

    init = sl.InitParameters()
    init.set_from_svo_file(input_svo)
    init.svo_real_time_mode = False
    init.depth_mode = sl.DEPTH_MODE.NONE   # cắt không cần depth -> nhanh

    cam = sl.Camera()
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        print("Không mở được file SVO2 để cắt")
        sys.exit(1)

    fps = cam.get_camera_information().camera_configuration.fps or 30
    total = cam.get_svo_number_of_frames()

    start_f = max(0, int(round(start_s * fps)))
    end_f = min(total - 1, int(round(end_s * fps)))
    if start_f >= end_f:
        print(f"Khoảng cắt không hợp lệ: frame {start_f} -> {end_f} (total={total})")
        cam.close()
        sys.exit(1)

    base = os.path.splitext(os.path.basename(input_svo))[0]
    out_path = os.path.join(clip_dir, f"{base}_clip_{start_f}_{end_f}.svo2")

    # Ghi ra SVO2 mới, dùng H.265 lossless để giữ chất lượng
    rec = sl.RecordingParameters(out_path, sl.SVO_COMPRESSION_MODE.H265)
    if cam.enable_recording(rec) != sl.ERROR_CODE.SUCCESS:
        print("Không bật được recording để cắt")
        cam.close()
        sys.exit(1)

    # Nhảy tới frame bắt đầu
    cam.set_svo_position(start_f)
    runtime = sl.RuntimeParameters()
    written = 0
    n_target = end_f - start_f + 1

    print(f"Cắt SVO: frame {start_f} -> {end_f} ({start_s:.1f}s -> {end_s:.1f}s), "
          f"{n_target} frame @ {fps}fps")

    while True:
        if cam.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            break
        # Ghi frame hiện tại (grab tự động ghi khi recording bật)
        written += 1
        cur = cam.get_svo_position()
        if cur >= end_f:
            break
        if written % 30 == 0:
            print(f"\r  cắt {written}/{n_target}", end="", flush=True)

    cam.disable_recording()
    cam.close()
    print(f"\nĐã lưu file cắt: {out_path} ({written} frame)")
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--start", required=True,
                    help="Thời điểm bắt đầu cắt, định dạng HH:MM:SS (vd 00:00:02)")
    ap.add_argument("--end", required=True,
                    help="Thời điểm kết thúc cắt, định dạng HH:MM:SS (vd 00:00:41)")
    ap.add_argument("--clip-dir", required=True,
                    help="Thư mục lưu file SVO2 đã cắt")
    ap.add_argument("--max-frames", type=int, default=0)
    args = ap.parse_args()

    # ---- BƯỚC 1: Cắt SVO2 theo start/end ----
    start_s = parse_hhmmss(args.start)
    end_s = parse_hhmmss(args.end)
    clipped_svo = clip_svo(args.input, args.clip_dir, start_s, end_s)

    # ---- BƯỚC 2: Convert file đã cắt sang MCAP ----
    init = sl.InitParameters()
    init.set_from_svo_file(clipped_svo)
    init.svo_real_time_mode = False
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD

    cam = sl.Camera()
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        print("Không mở được file SVO2")
        sys.exit(1)

    info = cam.get_camera_information()
    calib_raw = info.camera_configuration.calibration_parameters
    res = info.camera_configuration.resolution
    W, H = res.width, res.height
    fps = info.camera_configuration.fps or 30
    total = cam.get_svo_number_of_frames()
    serial = info.serial_number
    model = str(info.camera_model)
    fw = info.camera_configuration.firmware_version
    baseline = calib_raw.get_camera_baseline()
    print(f"Model={model} SN={serial} {W}x{H}@{fps}fps frames={total}")

    cam.enable_positional_tracking(sl.PositionalTrackingParameters())
    sensors_data = sl.SensorsData()

    enc_left = H265Encoder(W, H, fps)
    enc_right = H265Encoder(W, H, fps)
    enc_sbs = H265Encoder(W * 2, H, fps)

    img_left, img_right = sl.Mat(), sl.Mat()
    pose = sl.Pose()
    runtime = sl.RuntimeParameters()

    # Channel JSON cho 2 topic thiếu schema built-in
    imu_ch = Channel("/ego/imu", schema={"type": "object", "properties": IMU_SCHEMA["properties"]})
    sysinfo_ch = Channel("/ego/vio/system_info",
                         schema={"type": "object", "properties": ROBOTINFO_SCHEMA["properties"]})

    writer = foxglove.open_mcap(args.output, allow_overwrite=True)

    written = 0
    first_ns = None

    while True:
        err = cam.grab(runtime)
        if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            break
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"\nLỗi grab: {err}")
            break

        ns = cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
        ts = ns_to_ts(ns)

        if first_ns is None:
            first_ns = ns
            # system_info (JSON, ghi 1 lần)
            sysinfo_ch.log({
                "name": f"ZED {model}",
                "description": "ZED stereo camera converted from SVO2",
                "world_frame": "map",
                "body_frame": "zed_camera",
                "coordinate_convention": "RIGHT_HANDED_Z_UP_X_FWD",
                "serial_number": str(serial),
                "firmware_version": str(fw),
            }, log_time=ns)

            foxglove.log("/top-left-camera/camera-info",
                         make_calibration(ts, "top_left_camera", W, H, calib_raw.left_cam),
                         log_time=ns)
            foxglove.log("/top-right-camera/camera-info",
                         make_calibration(ts, "top_right_camera", W, H, calib_raw.right_cam),
                         log_time=ns)
            foxglove.log("/tf-static", FrameTransforms(transforms=[
                FrameTransform(
                    timestamp=ts,
                    parent_frame_id="top_left_camera",
                    child_frame_id="top_right_camera",
                    translation=Vector3(x=0.0, y=-baseline, z=0.0),
                    rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            ]), log_time=ns)

        # IMU (JSON)
        cam.get_sensors_data(sensors_data, sl.TIME_REFERENCE.IMAGE)
        imu = sensors_data.get_imu_data()
        ori = imu.get_pose().get_orientation().get()
        ang = imu.get_angular_velocity()
        lin = imu.get_linear_acceleration()
        imu_ch.log({
            "timestamp": ts_dict(ns),
            "frame_id": "zed_imu",
            "orientation": {"x": ori[0], "y": ori[1], "z": ori[2], "w": ori[3]},
            "angular_velocity": {
                "x": float(np.deg2rad(ang[0])),
                "y": float(np.deg2rad(ang[1])),
                "z": float(np.deg2rad(ang[2])),
            },
            "linear_acceleration": {"x": lin[0], "y": lin[1], "z": lin[2]},
        }, log_time=ns)

        # Pose VIO
        cam.get_position(pose, sl.REFERENCE_FRAME.WORLD)
        tr = pose.get_translation().get()
        q = pose.get_orientation().get()
        foxglove.log("/ego/vio/pose", FrameTransforms(transforms=[
            FrameTransform(
                timestamp=ts,
                parent_frame_id="map",
                child_frame_id="zed_camera",
                translation=Vector3(x=tr[0], y=tr[1], z=tr[2]),
                rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
            )
        ]), log_time=ns)

        # Ảnh -> H.265
        cam.retrieve_image(img_left, sl.VIEW.LEFT)
        cam.retrieve_image(img_right, sl.VIEW.RIGHT)
        bgr_l = img_left.get_data()[:, :, :3].copy()
        bgr_r = img_right.get_data()[:, :, :3].copy()
        bgr_sbs = np.hstack([bgr_l, bgr_r])

        for topic, enc, bgr, fid in [
            ("/top-left-camera/image-raw", enc_left, bgr_l, "top_left_camera"),
            ("/top-right-camera/image-raw", enc_right, bgr_r, "top_right_camera"),
            ("/side_by_side/image-raw", enc_sbs, bgr_sbs, "side_by_side"),
        ]:
            data = enc.encode_bgr(bgr)
            if not data:
                continue
            foxglove.log(topic, CompressedVideo(
                timestamp=ts, frame_id=fid, data=data, format="h265",
            ), log_time=ns)

        written += 1
        if written % 30 == 0:
            pct = written / total * 100 if total else 0
            print(f"\r  {written}/{total} ({pct:.1f}%)", end="", flush=True)

        if args.max_frames and written >= args.max_frames:
            break

    for topic, enc, fid in [
        ("/top-left-camera/image-raw", enc_left, "top_left_camera"),
        ("/top-right-camera/image-raw", enc_right, "top_right_camera"),
        ("/side_by_side/image-raw", enc_sbs, "side_by_side"),
    ]:
        data = enc.flush()
        if data:
            foxglove.log(topic, CompressedVideo(
                timestamp=ns_to_ts(first_ns), frame_id=fid, data=data, format="h265",
            ), log_time=first_ns)

    writer.close()
    cam.disable_positional_tracking()
    cam.close()
    print(f"\nXong! Ghi {written} frame vào {args.output}")


if __name__ == "__main__":
    main()