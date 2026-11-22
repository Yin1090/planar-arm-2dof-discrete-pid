# LabVIEW Reference Implementation (`FINAL2.vi`)

## English

### Compatibility & Architecture
- **Target Runtime**: **LabVIEW Base 22.3.7** (`NI.LV.All.SourceOnly`, `RSRC LVINLBVW`) — implemented exclusively with standard **While Loop**, **Shift Registers**, arithmetic primitives, `cos`/`sin`, `Build Array`, `Bundle`, and `Wait (ms)` blocks. No MathScript or Control & Simulation Module add-ons are required.
- **Controls (Front Panel Inputs)**:
  - `Theta1_ref`, `Theta2_ref` (DBL): Target joint angles in degrees (`0..40` slide on Front Panel; benchmark test `30.0°`).
  - `L1`, `L2` (DBL): Planar link lengths (`0..10` tank; benchmark `5.0`).
  - `Kp1`, `Ki1`, `Kd1` and `Kp2`, `Ki2`, `Kd2` (DBL): Independent discrete PID gain knobs (benchmark `Kp=15.0`, `Ki=0.1`, `Kd=0.2`).
  - `dt`, `dt 2` (DBL): Sampling time steps in seconds (`0..10` tank; benchmark `0.01 s`).
  - `Reset` (Boolean): Front Panel reset control.
  - `Stop` (Boolean): Loop termination button wired to the While Loop stop terminal.
- **Indicators (Front Panel Outputs)**:
  - `Theta1_actual`, `Theta2_actual` (DBL thermometers): Current joint angles in degrees.
  - `X`, `Y` (DBL gauges): End-effector Cartesian coordinates.
  - `Error 1`, `Error 2` (DBL dials) & `Error Chart` (2-trace `Waveform Chart`): Tracking errors over loop iterations.
  - `XY RobotArm` (`XY Graph`): 2-link planar arm polyline from base to end-effector.
- **Shift Register Memory & Timing**:
  - Each joint maintains 3 `DBL` Shift Registers initialized to `0.0` at While Loop entry: `theta_prev`, `e_prev`, and `I_prev`.
  - A `Wait (ms)` timer constant of **`20` ms** (`50 Hz` UI refresh) paces wall-clock execution without changing the mathematical step `dt = 0.01 s`.

### Observed VI Screenshot Quirks vs. Mathematical Specification
1. **`Reset` Terminal**: `Reset` exists on the Front Panel and VI terminal metadata, but its wire is left disconnected inside the Block Diagram (`FINAL2.vi`). In the Python engine (`planar_arm_pid`), `reset()` actively clears `(theta_deg, error_prev_deg, integral, step_index)` back to zero.
2. **`Error 1` / `Error 2` Wiring vs. Text Definition**: The mathematical equations define `Error_i = theta_i_ref - theta_i_actual` (`theta_new`), whereas the Block Diagram wires `e_i = theta_i_ref - theta_i_prev` (one-sample pre-update error) to `Error 1`/`Error 2` and `Error Chart`. By `k >= 100`, both definitions agree within `< 5e-6°`; Python records `error_deg = theta_ref - theta_new` and also exposes `error_pre_update` (`e`) for exact parity.
3. **`XY RobotArm` `ArrayY` & Front Panel Column Layout**: In the archived Block Diagram (`04_labview_block_diagram_pid1_fk.png`), `ArrayX` is `[0, x1, X]` while `ArrayY` is wired as `[y1, Y, 0]` instead of `[0, y1, Y]`, and on the Front Panel (`03_labview_front_panel.png`), `Theta2_actual` sits in the left column and `Theta1_actual` in the middle column. The Python simulator implements the exact geometric polyline `ArrayX = [0, x1, X]`, `ArrayY = [0, y1, Y]`.

---

## Tiếng Việt

### Tương thích & Kiến trúc VI
- **Nền tảng đích**: **LabVIEW Base 22.3.7** (`NI.LV.All.SourceOnly`, `RSRC LVINLBVW`) — xây dựng hoàn toàn bằng **While Loop**, **Shift Register**, các khối toán học cơ bản, `cos`/`sin`, `Build Array`, `Bundle` và `Wait (ms)`, không phụ thuộc MathScript hay Control & Simulation Module.
- **Các điều khiển đầu vào (Front Panel Controls)**:
  - `Theta1_ref`, `Theta2_ref` (DBL): Góc đặt tham chiếu theo độ (thang trượt `0..40`; thử nghiệm chuẩn `30.0°`).
  - `L1`, `L2` (DBL): Chiều dài hai khâu phẳng (`0..10`; thử nghiệm chuẩn `5.0`).
  - `Kp1`, `Ki1`, `Kd1` và `Kp2`, `Ki2`, `Kd2` (DBL): Núm xoay hệ số PID rời rạc độc lập cho từng khớp (chuẩn `Kp=15.0`, `Ki=0.1`, `Kd=0.2`).
  - `dt`, `dt 2` (DBL): Chu kỳ lấy mẫu rời rạc theo giây (chuẩn `0.01 s`).
  - `Reset` (Boolean): Nút đặt lại trạng thái trên Front Panel.
  - `Stop` (Boolean): Nút dừng nối trực tiếp vào đầu dừng của While Loop.
- **Các hiển thị đầu ra (Front Panel Indicators)**:
  - `Theta1_actual`, `Theta2_actual`: Góc khớp thực tế theo độ.
  - `X`, `Y`: Tọa độ đầu công tác trong mặt phẳng.
  - `Error 1`, `Error 2` & `Error Chart`: Sai số bám hai khớp theo từng chu kỳ lặp.
  - `XY RobotArm` (`XY Graph`): Đường gấp khúc 2 khâu từ gốc tọa độ tới đầu công tác.
- **Bộ nhớ Shift Register & Định thời vòng lặp**:
  - Mỗi khớp dùng 3 `Shift Register` kiểu `DBL` khởi tạo bằng `0.0` trước khi vào While Loop: `theta_prev`, `e_prev` và `I_prev`.
  - Khối `Wait (ms)` đặt hằng số **`20` ms** (`50 Hz`) để điều tiết tốc độ hiển thị giao diện mà không làm thay đổi bước tính `dt = 0.01 s`.

### Các điểm khác biệt quan sát được giữa ảnh chụp VI và mô hình toán học
1. **Chân `Reset` chưa nối dây trong Block Diagram**: Nút `Reset` có mặt trên Front Panel và metadata terminal của `FINAL2.vi` nhưng chưa được nối dây vào 3 Shift Register bên trong While Loop. Gói Python `planar_arm_pid` hiện thực đầy đủ `reset()` để xóa toàn bộ trạng thái về `0.0`.
2. **Dây nối `Error 1` / `Error 2` so với công thức**: Công thức toán học định nghĩa `Error_i = theta_i_ref - theta_i_actual` (`theta_new`), trong khi sơ đồ khối nối nhánh `e_i = theta_i_ref - theta_i_prev` (sai số trước khi cập nhật) ra `Error 1`/`Error 2` và `Error Chart`. Từ bước `k >= 100`, hai giá trị này chênh nhau nhỏ hơn `5e-6°`; bản Python ghi nhận `error_deg = theta_ref - theta_new` đồng thời cung cấp thêm `error_pre_update` (`e`) để đối chiếu chính xác từng bước.
3. **Nối dây `ArrayY` trên `XY RobotArm` & vị trí nhãn Front Panel**: Trong ảnh chụp Block Diagram (`04_labview_block_diagram_pid1_fk.png`), `ArrayX` nhận `[0, x1, X]` còn `ArrayY` nối `[y1, Y, 0]` thay vì `[0, y1, Y]`, và trên Front Panel (`03_labview_front_panel.png`) nhãn `Theta2_actual` đặt ở cột trái còn `Theta1_actual` ở cột giữa. Trình mô phỏng Python sử dụng đúng tọa độ hình học chuẩn `ArrayX = [0, x1, X]`, `ArrayY = [0, y1, Y]`.
