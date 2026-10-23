# Kiểm tra V3.16.11

- Phạm vi: đổi cửa sổ gắp từ 250–300 mm sang **200–250 mm**, đồng bộ thông báo, mô phỏng và giá trị khởi tạo firmware. Giữ ROI10×10, D4 quà, kẹp2000 us.
- Lượt đầu: 138 ca liên quan khoảng cách, AUTO, điều khiển tay, quà D4 và firmware host; **132 đạt, 6 lỗi kiểm thử**. Các lỗi do fixture/kỳ vọng còn ngưỡng cũ, chuỗi phiên bản cũ và kiểm thử cấu hình JSON lịch sử không tương thích với bộ cấu hình tổng hiện tại.
- Đã cập nhật fixture: 225 mm là mẫu giữa khoảng, 190 mm dưới khoảng, 200/250 mm là biên. Mô phỏng giảm từ900 mm cần ba nhịp để tới khoảng mới, thay vì hai; tình huống mất box tương ứng hai nhóm ba frame mất.
- Lượt kiểm tra lại: 1 ca cấu hình đạt; nhóm16 ca đạt15, còn1 kỳ vọng số frame mất cũ; sửa kỳ vọng rồi chạy lại cả2 chu trình đầy đủ, **2/2 đạt**. Không đổi logic sản xuất để ép kiểm thử qua.
- `v31611_verified_cases.xml` tổng hợp kết quả mới nhất của **138 ca**, không còn lỗi; đây không phải một lượt chạy toàn bộ bộ kiểm thử482 ca. Hai tên tham số của biên cũ được ánh xạ sang biên mới để tổng hợp đúng cùng ca.
- Bao gồm kiểm tra200/225/250 mm cho phép chuyển gắp,199/251 mm không cho gắp; VL53 lỗi/chưa có vẫn tiếp cận theo nhịp; STOP; gắp/thả mô phỏng; mã firmware thật biên dịch và chạy với stub host.
- Python compileall đạt; JavaScript giao diện qua node --check; check_system xuất đúng200–250 mm,ROI10×10,D4 và kẹp2000 us.
- Tất cả tệp nguồn của ZIP trước vẫn hiện diện. `.env` và thư viện Arduino giữ nguyên byte; chi tiết trong `integrity_v31611.json`.
- **Chưa chạy trên robot thật**, chưa biên dịch bằng AVR toolchain của Arduino IDE. Kiểm thử host không thay thế việc nạp và đo thực tế.
