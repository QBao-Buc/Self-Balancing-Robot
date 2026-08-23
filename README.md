# Self-Balancing Robot (Dự án Nhúng Full-Stack)

Một dự án robot độc lập được thiết kế Full-Stack hoàn chỉnh: từ mạch cứng PCB tùy chỉnh, firmware nhúng C/C++, đến các thuật toán điều khiển PID trên vi điều khiển STM32.

---

## 📌 Tổng quan dự án

Dự án này tập trung vào việc thiết kế và chế tạo một robot 2 bánh tự cân bằng hoàn toàn từ con số 0. Hệ thống tích hợp toàn diện giữa thiết kế phần cứng, dung hợp cảm biến (sensor fusion), lý thuyết điều khiển rời rạc và lập trình hệ thống nhúng.

* **Vi điều khiển chính:** STM32F103C8T6 (Blue Pill)
* **Cảm biến:** MPU6050 (Góc nghiêng / Gia tốc 6-axis)
* **Mạch công suất:** TB6612FNG + Động cơ Encoder DC
* **Nguồn cấp:** Pin LiPo 3S (11.1V - 12.6V) qua mạch Hạ áp LM2596/Buck

---

## 🛠️ Kiến trúc Hệ thống & Đặc tính Kỹ thuật

* **Vi điều khiển:** Dòng STM32F103.
* **Phần cứng & PCB:** Thiết kế mạch nguyên lý & PCB 2 lớp tùy chỉnh, tích hợp khối cảm biến IMU MPU6050 và mạch điều khiển động cơ.
* **Firmware nhúng:** Lập trình C/C++ sử dụng thư viện STM32 HAL/Register, Ngắt bộ định thời (Timer Interrupts), và các ngoại vi I2C/PWM.
* **Chiến lược điều khiển:**
  * Dung hợp cảm biến thông qua Bộ lọc bù (Complementary Filter) / Bộ lọc Kalman để ước lượng góc nghiêng chính xác.
  * Thuật toán **PID 2 vòng kép (Cascade PID)** để cân bằng góc nghiêng, kiểm soát vận tốc và vị trí.
* **Cơ khí:** Thân vỏ khung xe in 3D tùy chỉnh được thiết kế trên SolidWorks.

---

## 📂 Cấu trúc Repository

```plaintext
Self-Balancing-Robot/
├── .gitignore
├── LICENSE
├── README.md
├── CHANGELOG.md
├── firmware/                 # Mã nguồn lập trình STM32 (CubeMX + Keil C)
│   ├── Core/
│   ├── Drivers/
│   ├── MDK-ARM/              # Project Keil C (.uvprojx)
│   ├── MyLib/                # Thư viện tự viết (robot_debug.c/h)
│   └── STM32F103_BalanceRobot.ioc
├── hardware/                 # Sơ đồ nguyên lý & PCB
│   ├── schematic/            # Bản vẽ nguyên lý (.PDF)
│   ├── wiring/               # Sơ đồ đấu nối dây (.PNG)
│   ├── pcb/                  # Layout PCB (.PDF)
│   └── BOM.md                # Danh mục linh kiện chi tiết
└── mechanical/               # Thiết kế cơ khí & Thân vỏ
    ├── cad/
    │   └── stl_print/        # File .STL phục vụ in 3D
    ├── drawings/             # Bản vẽ 3D tổng thể (.STEP) & bản vẽ 2D (.PDF)
    └── README.md             # Hướng dẫn in 3D & chế tạo

🚀 Tính năng chính của Robot
Giao tiếp & Debug: Gửi/nhận dữ liệu không dây qua Bluetooth HC-05, theo dõi thông số thực thời trên máy tính qua phần mềm Tera Term 5 (MyLib/robot_debug).

Cơ chế bảo vệ an toàn:
Dead Angle Protection: Tự động ngắt động cơ khi xe nghiêng quá góc cho phép (tránh trôi xe hoặc quá tải khi đổ).
Bảo vệ điện áp/Dòng: (Đang hoàn thiện)
Tính năng phụ trợ:
Điều khiển còi kèn (Buzzer) phát tín hiệu cảnh báo.
Hiệu ứng đèn LED trạng thái (nháy LED báo lỗi / chế độ hoạt động).
(Lưu ý: Các tính năng trên đang tiếp tục được tối ưu và hoàn thiện trong code).

📋 Danh mục chi tiết cần chuẩn bị (BOM)
Để xem danh sách đầy đủ tất cả linh kiện điện tử, động cơ, cảm biến và vật tư cơ khí cần chuẩn bị cho dự án:
👉 Mời bạn truy cập vào file: hardware/BOM.md
🛠️ Sơ đồ nguyên lý & Bảng chân đấu nối (Pinout)
1. Sơ đồ nguyên lý (Schematic)
(Phần này để trống - Cập nhật sơ đồ nguyên lý hoặc đường dẫn bản vẽ sau)

2. Sơ đồ đấu nối phần cứng
Chi tiết đấu nối giữa STM32, TB6612, Động cơ Encoder và Nguồn:

📖 Hướng dẫn lắp ráp & Vận hành
1. Phần cơ khí & Thân vỏ
In tất cả các file .STL nằm tại mechanical/cad/stl_print/ và xem bản vẽ lắp ráp tổng thể .STEP tại mechanical/drawings/.

2. Phần cứng & Nguồn
(Phần này để trống - Cập nhật hướng dẫn đấu nối chi tiết)

3. Nạp code Firmware
Mở project tại firmware/MDK-ARM/STM32F103_BalanceRobot.uvprojx bằng Keil uVision 5.

Build và nạp code xuống STM32 bằng mạch nạp ST-Link V2.

📄 Giấy phép (License)
Dự án được phân phối dưới giấy phép MIT License.