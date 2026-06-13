# Convert ZED SVO2 → MCAP (Foxglove)

Hướng dẫn cài đặt và sử dụng bộ script convert file `.svo2` từ camera ZED sang định dạng `.mcap` để xem trong Foxglove Studio, kèm script xuất metadata.

## Yêu cầu hệ thống

- WSL2 (Ubuntu 22.04) hoặc Linux
- GPU NVIDIA + driver hỗ trợ CUDA (đã xác nhận chạy: RTX 3060, CUDA 12.8)
- ZED SDK 4.1+ (bắt buộc để đọc định dạng SVO2)
- Camera ZED có IMU (ZED 2/2i/Mini/X/X Mini) để có dữ liệu IMU thật

## Cài đặt

### 1. ZED SDK + Python API

Cài ZED SDK từ trang stereolabs (chọn đúng bản Ubuntu + CUDA), trong quá trình cài chấp nhận cài Python API (`pyzed`). Kiểm tra:

```bash
python -c "import pyzed.sl as sl; print('pyzed OK')"
```

### 2. Thư viện Python

```bash
pip install foxglove-sdk av numpy mcap pyyaml
```

Thư viện `av` (PyAV) cần build kèm `libx265` để encode H.265 (thường có sẵn trong wheel chuẩn). Kiểm tra:

```bash
python -c "import av; print('libx265' in [c for c in av.codecs_available])"
# Mong đợi: True
```

## Sử dụng

### Bước 1 — Convert SVO2 sang MCAP

```bash
# Test nhanh với 100 frame trước
python mcap_converter.py video.svo2 video.mcap --max-frames 100

# Chạy đầy đủ
python mcap_converter.py video.svo2 video.mcap
```

Tham số:

- `input` : đường dẫn file `.svo2`
- `output` : đường dẫn file `.mcap` xuất ra
- `--max-frames N` : chỉ xử lý N frame đầu (0 = toàn bộ)

### Bước 2 — Xuất metadata

```bash
python mcap_to_metadata.py video.mcap metadata_sample.yaml
```

Xuất file YAML liệt kê từng topic kèm schema, encoding, count và giá trị mẫu từ message đầu tiên.

## Các topic xuất ra

| Topic | Schema | Nguồn dữ liệu |
|---|---|---|
| `/ego/imu` | JSON (IMUMeasurement) | Sensor IMU thật từ SVO |
| `/ego/vio/pose` | foxglove.FrameTransforms | ZED Positional Tracking |
| `/ego/vio/system_info` | JSON (RobotInfo) | Thông tin camera |
| `/side_by_side/image-raw` | foxglove.CompressedVideo | Ảnh left+right ghép (H.265) |
| `/top-left-camera/image-raw` | foxglove.CompressedVideo | Ảnh left (H.265) |
| `/top-right-camera/image-raw` | foxglove.CompressedVideo | Ảnh right (H.265) |
| `/top-left-camera/camera-info` | foxglove.CameraCalibration | Calibration trái |
| `/top-right-camera/camera-info` | foxglove.CameraCalibration | Calibration phải |
| `/tf-static` | foxglove.FrameTransforms | Baseline trái→phải |

Lưu ý: `/ego/imu` và `/ego/vio/system_info` dùng JSON schema tự định nghĩa vì foxglove-sdk bản đang dùng không có schema built-in `IMUMeasurement` và `RobotInfo`. Các topic còn lại dùng schema protobuf built-in.

## Tăng tốc

Quá trình chậm chủ yếu do encode H.265 và tính depth mỗi frame. Mặc định script dùng encoder CPU `libx265` preset `fast`. Các cách tăng tốc, từ hiệu quả nhất:

1. **Preset x265 nhanh hơn** — đổi `"preset": "fast"` thành `"preset": "ultrafast"` trong `H265Encoder`. Nhanh hơn nhiều, file lớn hơn không đáng kể.
2. **Depth mode nhẹ hơn** — đổi `init.depth_mode` sang `sl.DEPTH_MODE.PERFORMANCE` thay vì `NEURAL`. Positional tracking vẫn chạy, chỉ kém chính xác hơn chút.
3. **GPU encode (NVENC)** — nếu PyAV có build NVENC (kiểm tra: `python -c "import av; print([c for c in av.codecs_available if 'nvenc' in c])"`), đổi `libx265` thành `hevc_nvenc` với options `{"preset": "p1", "tune": "ll"}`. Nhanh nhất nhưng phụ thuộc phần cứng/PyAV.
4. **Giảm độ phân giải** — đặt `init.camera_resolution = sl.RESOLUTION.HD720` khi mở file.

## Cấu hình kỹ thuật

- Hệ tọa độ: `RIGHT_HANDED_Z_UP_X_FWD`, đơn vị mét
- Angular velocity của IMU đã đổi từ deg/s sang rad/s
- Video encode H.265 bằng `libx265` (CPU), preset `fast`
- Frame trong buffer encoder được flush ở cuối để không mất frame

## Xử lý lỗi thường gặp

| Lỗi | Cách khắc phục |
|---|---|
| `FileExistsError` khi mở mcap | File đã tồn tại; script dùng `allow_overwrite=True`, hoặc `rm -f video.mcap` trước khi chạy |
| `unexpected keyword argument` ở schema | Tên field khác giữa các bản SDK; chạy `help(<Schema>)` để xem tên đúng |
| `Timestamp` lỗi tham số | Bản SDK dùng `sec`/`nsec` thay vì `seconds`/`nanos` |
| `libx265` không có | PyAV thiếu build libx265; cài lại PyAV bản đầy đủ |
| Chạy quá chậm | Xem mục Tăng tốc |

## Kiểm tra kết quả

M�� file `.mcap` trong Foxglove Studio (app desktop hoặc web), xác nhận đủ 9 topic và ảnh H.265 hiển thị được. Nên kiểm tra với bản `--max-frames 100` trước khi chạy full.
