# Waste Robot V3.16.32 — Jetson Nano USB CH341

- Cổng mặc định `/dev/ttyUSB0`, thiết lập trực tiếp trong `python/.env` (không cần tự tạo).
- Nếu cổng đổi sang `/dev/ttyUSB1` hoặc `/dev/ttyACM0`, lệnh Kết nối lại sẽ tìm cổng USB Serial khả dụng. Không sử dụng UART nội bộ `/dev/ttyAMA0`.
- Để kiểm tra: `python3 python/main.py --diagnose`, `python -m serial.tools.list_ports -v`.
- Nếu bị Permission denied: `sudo usermod -aG dialout $USER`, rồi đăng xuất và đăng nhập.
- Đóng Arduino IDE Serial Monitor trước khi Python kết nối.
- Chạy từ VS Code: `python3 python/main.py`.
- LiveKit/OpenCV và logic AUTO/D18/D19 giữ nguyên.
- Không tự tiếp tục chu trình cơ khí không an toàn sau khi COM rớt; vẫn dùng checkpoint và xác nhận tư thế.
- **Chỉ kết nối nếu chắc chắn USB Serial đang nối đúng bo robot**. Nếu cắm nhiều bộ CH341, hãy đặt ARDUINO_PORT cụ thể hoặc sử dụng symlink udev cố định.
