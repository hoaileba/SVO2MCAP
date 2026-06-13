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

Bộ gồm 2 file: `mcap_converter.py` (cắt + convert) và `export_metadata.py` (xuất metadata).

### Bước 1 — Cắt SVO2 và convert sang MCAP

Script luôn cắt SVO2 theo khoảng thời gian rồi mới convert. File SVO2 đã cắt được lưu vào thư mục chỉ định qua `--clip-dir`.

```bash
# Cắt từ giây thứ 2 đến giây 41, lưu clip vào ./clips rồi convert
python mcap_converter.py video.svo2 video.mcap \
  --start 00:00:02 --end 00:00:41 --clip-dir ./clips

# Test nhanh: chỉ convert 100 frame đầu của đoạn đã cắt
python mcap_converter.py video.svo2 video.mcap \
  --start 00:00:02 --end 00:00:41 --clip-dir ./clips --max-frames 100
```

Tham số:

- `input` : đường dẫn file `.svo2` gốc
- `output` : đường dẫn file `.mcap` xuất ra
- `--start` : thời điểm bắt đầu cắt, định dạng `HH:MM:SS` (hỗ trợ cả `MM:SS`, `SS`) — bắt buộc
- `--end` : thời điểm kết thúc cắt, định dạng `HH:MM:SS` — bắt buộc
- `--clip-dir` : thư mục lưu file SVO2 đã cắt (tự tạo nếu chưa có) — bắt buộc
- `--max-frames N` : chỉ convert N frame đầu của đoạn đã cắt (0 = toàn bộ)

File clip được đặt tên `<tên_gốc>_clip_<frame_start>_<frame_end>.svo2` trong `--clip-dir`.

### Bước 2 — Xuất metadata

```bash
python export_metadata.py video.mcap metadata_sample.yaml
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
| Lỗi `H265` ở recording khi cắt | Đổi `SVO_COMPRESSION_MODE.H265` sang `H264` hoặc `LOSSLESS` |
| File clip rỗng | Kiểm tra `--start`/`--end` nằm trong thời lượng video; start phải nhỏ hơn end |
| Chạy quá chậm | Xem mục Tăng tốc |

## Kiểm tra kết quả

Mở file `.mcap` trong Foxglove Studio (app desktop hoặc web), xác nhận đủ 9 topic và ảnh H.265 hiển thị được. Nên kiểm tra với bản `--max-frames 100` trước khi chạy full.
