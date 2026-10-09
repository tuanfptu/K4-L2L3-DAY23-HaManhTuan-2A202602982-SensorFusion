# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Ha Manh Tuan
- MSSV: 2A202602982
- Email:
- Link repo (fork): https://github.com/tuanfptu/K4-L2L3-DAY23-HaManhTuan-2A202602982-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): Ghi trên LMS cùng link repo

## Tóm tắt kết quả

- `fusion_mode=compare`, `frames=[0, 198]`, segment `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `seed=0`.
- Detection: precision `0.9701`, recall `0.7004`, TP/FP/FN = `519/16/222`.
- LiDAR tracking: RMSE `0.150323 m`, matches `502`, sum_sq_err `11.343646 m²`, ghosts `0`, misses `239`, mean confirmed tracks `2.5226`.
- Fused tracking: RMSE `0.135867 m`, matches `502`, sum_sq_err `9.266808 m²`, ghosts `0`, misses `239`, mean confirmed tracks `2.5226`.
- Cả hai mode ghép được cùng 502 cặp, không có ghost và bỏ lỡ 239 nhãn xe hợp lệ. Camera giảm RMSE khoảng `0.014456 m` (xấp xỉ `9.6%`) mà không thay đổi số matches/ghost/miss; các số được lấy từ `student/artifacts/metrics.json` và log JSONL tương ứng.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. **Đo LiDAR và camera trong EKF:** LiDAR đo tâm 3D trong hệ tọa độ cảm biến, `z=[x,y,z]`, `R` là covariance theo mét². Camera đo pixel `z=[u,v]`, dùng phép chiếu pinhole từ tọa độ camera và `R=diag(σ_i²,σ_j²)` theo pixel². Mỗi sensor có hàm đo/Jacobian riêng; EKF dùng chúng để tính innovation và cập nhật cùng trạng thái 6D.
2. **Mahalanobis gating:** Dùng `d²=γᵀS⁻¹γ` để xét độ lệch so với bất định tổng hợp của track và measurement; cổng χ² theo số chiều đo loại cặp không hợp lý trước khi greedy matching. Vì chuẩn hóa theo `S`, cùng một sai lệch được chấp nhận rộng hơn khi bất định lớn; Euclidean không xét covariance.
3. **Track-then-fuse:** Runner dự đoán track một lần mỗi frame, chạy association/update LiDAR, rồi association/update camera lên các track hiện có. Có thể đối chiếu các record `mode=lidar` và `mode=fused` trong `grade_run.log`; phần khởi tạo/xác nhận/xóa chỉ ở lượt LiDAR. Camera lab là tâm hộp 2D ground-truth FRONT cộng nhiễu seeded, không phải detector ảnh.
4. **Lệch calibration camera:** Extrinsic/intrinsic sai làm dự đoán pixel `h(x)` lệch có hệ thống, khiến innovation/residual camera thường lớn hoặc lệch theo một hướng. Nhiều residual vượt cổng χ² sẽ bị từ chối; nếu vẫn lọt cổng, update có thể kéo trạng thái track sai và làm tăng sai số.
5. **Sensor tường minh ở frame rỗng:** Khi `meas_list` rỗng, không thể suy ra lượt xử lý là LiDAR hay camera từ measurement. Sensor tường minh giúp luôn gọi quản lý đúng lượt: LiDAR có thể ghi miss cho track đang trong FOV và đánh giá xóa; camera rỗng không được tính miss tồn tại. LiDAR tạo track từ measurement chưa ghép và quyết định score/lifecycle; camera chỉ tinh chỉnh EKF.
6. **Vòng đời track:** Track mới khởi tạo với score `1/window`; mỗi hit LiDAR cộng `1/window` (tối đa 1), mỗi miss trong FOV trừ `1/window`. Track được xác nhận khi `score > confirmed_threshold`; track đã confirmed vẫn giữ trạng thái sau miss. Xóa nếu phương sai ngang `P[0,0]` hoặc `P[1,1]` vượt `max_P`, hoặc track confirmed có score `< delete_threshold`, hoặc track chưa confirmed có score `<= 0`.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- **Trực quan hóa (+3):** `student/bonus/bev_tracks.png` cho thấy các track ID trên BEV qua frame 0–198; `camera_update_effect.png` so sánh cùng track ID 14 ở frame 148 ngay sau lượt LiDAR và sau camera. Camera update dịch trạng thái XY khoảng `0.428 m`; `tracks_cvat.json` được xuất bằng `fusion_lab.export_cvat.export_tracks_json`.
- **Phân tích calibration (+4):** Giữ nguyên segment, seed 0 và pipeline, cộng sai lệch tịnh tiến ngang vào `veh_to_sens[1,3]` của camera. Bảng và dữ liệu chi tiết ở `student/bonus/calibration_results.json`. Innovation norm gồm residual ở bước thử association và residual được tính lại cho EKF update đã ghép; median Mahalanobis và tỷ lệ qua cổng tính trên candidate pairs.

| Lệch ngang extrinsic (m) | Innovation norm median / P95 (px) | Median `d²` | Qua cổng χ² | RMSE fused (m) |
|---:|---:|---:|---:|---:|
| 0.00 | 235.8 / 775.5 | 691.0 | 7.52% | 0.1359 |
| 0.25 | 243.4 / 780.8 | 728.0 | 7.52% | 0.2004 |
| 0.50 | 270.4 / 800.2 | 778.3 | 3.69% | 0.2476 |
| 1.00 | 303.8 / 829.0 | 919.4 | 1.28% | 0.1663 |

Khi lệch ngang tăng, residual pixel và `d²` trung vị tăng, còn tỷ lệ candidate qua cổng giảm; một số phép đo sai vẫn nằm trong cổng nên RMSE tăng ở 0.25–0.50 m. Ở 1.00 m, gating loại gần 99% candidate, khiến RMSE giảm về gần baseline LiDAR `0.1503 m` nhưng vẫn cao hơn. Cả ba mức vẫn có 502 matches, 0 ghost và 239 misses. `tracks_cvat.json` đã được xuất; chưa có ảnh xác nhận import vào CVAT nên không khai claim bonus CVAT (+3).

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): ChatGPT (Codex).
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ triển khai/rà soát E–H, trực quan hóa track, chạy và diễn giải thử nghiệm calibration; báo cáo dùng số liệu chạy thực tế.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Chạy `pytest student/tests -q` (128 passed); chạy `--fusion compare --seed 0` trên frame 0–198; chạy ba mức lệch extrinsic, lưu kết quả; đối chiếu metrics với JSONL.

## Checklist nộp

- [ ] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [ ] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [ ] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [ ] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [ ] Đã điền đủ file này, gồm khai báo AI
- [ ] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [ ] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
