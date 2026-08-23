# 🛠️ Thiết kế Cơ khí & Thân vỏ (Mechanical)

Thư mục này chứa toàn bộ tài liệu thiết kế cơ khí, mô hình 3D linh kiện, file in 3D và bản vẽ lắp ráp cho robot tự cân bằng.

---

## 📂 Cấu trúc thư mục & Giải thích chức năng

```plaintext
mechanical/
├── README.md               # Hướng dẫn chi tiết phần cơ khí (File này)
├── cad/                    # Thư mục thiết kế mô hình 3D (CAD Models)
│   ├── circuit_parts/      # Mô hình 3D các linh kiện điện tử & bo mạch
│   │   ├── battery/        # Mô hình 3D Pin LiPo
│   │   ├── buck/           # Mô hình 3D mạch hạ áp Buck
│   │   ├── dong_co/        # Mô hình 3D động cơ DC Encoder
│   │   ├── hc05/           # Mô hình 3D Module Bluetooth HC-05
│   │   ├── mpu/            # Mô hình 3D Cảm biến MPU6050
│   │   └── pcb/            # Mô hình 3D Bo mạch PCB
│   └── stl_print/          # Các file định dạng .STL dùng để in 3D
└── drawings/               # Bản vẽ tổng thể & Kỹ thuật
    ├── *.STEP              # File lắp ráp 3D tổng thể (Assembly STEP)
    └── *.PDF               # Bản vẽ 2D / Sơ đồ chi tiết kích thước
```

## 📖 Hướng dẫn sử dụng từng phần

### 1. Thư mục cad/circuit_parts/ (Mô hình linh kiện điện tử)
Chức năng: Chứa mô hình 3D (STEP/SolidWorks) của từng linh kiện điện tử thực tế.

Cách dùng: Sử dụng khi bạn muốn tự thiết kế lại khung xe/PCB bằng các phần mềm CAD (SolidWorks, Fusion 360, Inventor). Hãy Import các file này vào phần mềm thiết kế để lấy chính xác kích thước và vị trí bắt ốc.

### 2. Thư mục cad/stl_print/ (File in 3D)
Chức năng: Chứa các file định dạng .STL của thân vỏ, gá động cơ, khay pin đã được tối ưu cho máy in 3D.

Cách dùng:

Mở phần mềm Cắt lát (Slicer) như Cura, PrusaSlicer, hoặc Bambu Studio.

Import tất cả file .STL trong thư mục này.

Tiến hành Cắt lát (Slice) và xuất file .gcode ra thẻ nhớ để đưa vào máy in 3D.

Khuyến nghị thông số in: Infill 20% - 30% (dạng Gyroid/Grid), độ dày thành (Wall thickness) >= 1.2mm, vật liệu in PLA hoặc PETG.

### 3. Thư mục drawings/ (Bản vẽ tổng thể)
Chức năng: Chứa bản vẽ phối cảnh 3D tổng thể (.STEP) và bản vẽ kỹ thuật 2D (.PDF).

Cách dùng:

File .STEP: Mở bằng phần mềm CAD để xem tổng thể robot sau khi lắp ráp đầy đủ khung, động cơ, mạch và pin.

File .PDF: Xem trực tiếp để nắm kích thước tổng thể robot và vị trí lắp ghép chuẩn.

## 👉 Để quay lại hướng dẫn chung của toàn bộ dự án, truy cập Trang README chính.