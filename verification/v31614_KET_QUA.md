# Kiểm tra V3.16.14

## Thay đổi

- Xung đóng kẹpD3 đổi2000→2300us trong `robot_params.py` và JSONdự phòng; mở600us và các khớp khác giữ nguyên.
- `LOCAL_OPERATOR_UI=True` mở máy chủ camera/control trên127.0.0.1, dùng lại brain/controller/vision hiện có; không mở camera hay COM lần hai.
- Chỉ mở trình duyệt sau khi máy chủ đã sẵn sàng. Cổng bận thì không tự mở trang vào một phiên khác. Khi tắtPython, máy chủ nhận tín hiệu thoát và đóng cổng.
- Giao diệncamera và điều khiển cùng trang, mốcAUTO, fullscreen/fallback, quàD4, STOP, nhật ký thu gọn và CSSresponsive. Mất ảnh thì che ảnh cũ và hiện thông báo.

## Kết quả

- Lượt131ca liên quan: **131đạt** trong96.76giây; gồmmanual, AUTO, IRsớm, hạcuối, VL53, quàD4 và vòng gắp/thả mô phỏng với xungđóng2300us.
- Sau hoàn thiện shutdown, nhóm9ca camera/server: **9đạt** trong2.01giây; gồm chạy máy chủHTTP loopback thật, /control và preview dùng chung, không tự phát lệnh cơ khí, tự mở trình duyệt đúng một lần khi đã sẵn sàng, cổng bận không mở nhầm, kiểm tra xác thực lệnh và xungD3=2300us.
- Tổng hợp kết quả mới nhất **132ca liên quan đều đạt**, không phải một lượt chạy toàn bộ bộ kiểm thử lịch sử. XML ở `v31614_verified_cases.xml`.
- BộharnessJavaScript thuầnNode đạt: tải trang không gửi lệnh cơ khí; hiển thị2300; callbackcamera, đổiquà, fullscreenfallback, STOP, bật/tắt mốc, mấtảnh/phụchồi và CSSresponsivehooks. Không phải kiểm thử hiển thị pixel bằng trình duyệt.
- HTML không trùngID và các thẻdiv cân bằng; JavaScript qua kiểm tra cú pháp; Pythoncompileall và check_system đạt.
- Có cảnh báo deprecation từ FastAPI/StarletteTestClient về httpx; không có lỗi kiểm thử và không đổi thư viện production vì cảnh báo này.

## Giới hạn

- Không xác nhận trực quan bố cục trong trình duyệt: bộChromium QA không tải được gói browser hợp lệ, trình duyệt cloud không truy cập được localhost của môi trường dựngmã. Đã thay bằng kiểm thửHTTP thực và harnessJavaScript, không tuyên bố đã kiểm tra ảnh chụp màn hình desktop/mobile.
- Chưa thử webcam, browser mặc định hoặcPython3.14 trên máyWindows của người dùng; chưa chạy trênrobot thật. Kiểm thử sử dụngPython3.12 và mô phỏng cơ khí.
- Firmware, thư việnArduino, tệp `.env` giữ nguyên byte; tất cả tệp của ZIPtrước vẫn hiện diện. Xem `integrity_v31614.json`.
