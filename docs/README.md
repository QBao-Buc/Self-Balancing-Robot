cat << 'EOF' > docs/README.md
# 📚 Tài liệu tham khảo & Nhật ký phát triển (Docs)

Thư mục này chứa toàn bộ các tài liệu kỹ thuật, datasheet linh kiện, bản đồ gán chân (Pinout Mapping) và tài liệu nhật ký ghi chép quá trình nghiên cứu, chế tạo robot tự cân bằng.

---

## 📂 Cấu trúc lưu trữ

```plaintext
docs/
├── README.md               # Trang hướng dẫn & Tổng quan thư mục tài liệu
├── datasheets/             # Tập hợp Datasheet PDF của các linh kiện
├── pinout/                 # Sơ đồ kết nối chân & Bảng tra cứu Pinout chi tiết
└── handbook/               # Nhật ký ghi chép, tài liệu tính toán & Debug
```

---

## 📖 Danh mục tài liệu chi tiết

### 1. 📄 Datasheets (`docs/datasheets/`)
Tài liệu kỹ thuật gốc từ nhà sản xuất phục vụ tra cứu thông số điện áp, dòng điện, thanh ghi (registers) và các chuẩn giao tiếp:
* **STM32F103C8T6:** Datasheet vi điều khiển ARM Cortex-M3 & Reference Manual (`RM0008`).
* **MPU6050:** Product Specification & Register Map (Bộ lọc dung hợp cảm biến IMU 6-axis).
* **TB6612FNG:** Datasheet mạch cầu H điều khiển động cơ DC.
* **HC-05:** Tài liệu cấu hình Bluetooth Module & Tập lệnh AT Command.
* **LM2596 / Buck Converter:** Datasheet mạch nguồn hạ áp.

---

### 2. 📌 Pinout & Sơ đồ đấu nối (`docs/pinout/`)
* **Sơ đồ gán chân tổng thể:** Bảng tra cứu chân kết nối giữa STM32F103C8T6 với các ngoại vi (I2C MPU6050, Timer Encoder, PWM TB6612, UART Bluetooth).
* **Sơ đồ mạch nguyên lý & Wiring:** Bản vẽ kết nối dây thực tế giúp người dùng có thể lắp ráp nhanh chóng.

---

### 3. 📝 Handbook & Nhật ký dự án (`docs/handbook/`)
Tài liệu hướng dẫn chuyên sâu từ con số 0:
* **Lý thuyết & Thuật toán:** Giải thuật lọc bù (Complementary Filter), Kalman Filter và thuật toán điều khiển PID 2 vòng kép (Cascade PID).
* **Kinh nghiệm cân chỉnh (PID Tuning):** Quy trình thực tế để chỉnh các thông số $K_p, K_i, K_d$.
* **Xử lý sự cố (Troubleshooting):** Tổng hợp các vấn đề thường gặp (nhiễu IMU, trôi góc 0, sụt áp do động cơ, nhiễu nguồn...) và cách khắc phục.

---

👉 *Để xem hướng dẫn nạp code và chạy dự án, vui lòng quay lại [Trang README chính](../README.md).*
