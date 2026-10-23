# Kiểm tra V3.16.13

- Luồng mới: khi đã khóa vật/BIN, IRD19 nhận sớm cho phép dừng bánh và hạ hết tớiP1=2310/P10=650, kể cả khiVL53 chưa vào20–25cm. Sau đó đọc lạiIR hiện tại; không dùng tín hiệu sớm làm bằng chứng để đóng kẹp.
- NếuIR cuối không nhận: nhích160ms thực thi mỗi nhịp, dừng kiểm tra lại; giữ giới hạn12nhịp/15giây, xác nhậnIR ổn định300ms và các kiểm tra sau nâng/thả.
- **161ca hồi quy đạt** trong172.83giây: AUTO box/VL53, IR sau hạ, cửa sổ20–25cm, VL53 lỗi/chưa có/mẫu cũ, phục hồi box, điều khiển tay, luồng pose_profile cũ và firmware qua trình biên dịchhost.
- **13ca mới đạt** trong20.27giây: IR sớm tạiSEARCH/1550/2050, IR vớiVL53 dưới ngưỡng, nhận khi hạ/tiến/căn ngang, hủy camera chờ trước hạ, IR thoáng qua rồi mất không cho gắp, không khóa vật thì không hạ, STOP, D18 không bị nhầm thànhD19, và chu trình dùng tác vụ tay của phần mềm để gắp/nâng/thả mô phỏng.
- Tổng hợp **174ca liên quan đều đạt**, hai lượt chạy, ở `v31613_verified_cases.xml`. Không chạy lại toàn bộ bộ kiểm thử lịch sử.
- Python compileall, JavaScript node --check và check_system đạt.
- Đã rà lại mã để việc từ chối bánh doD18 không bị chuyển nhầm thành nhánhIR sớm; chỉ D19 nhận mới cho chuyển nhánh. Các kiểm traSTOP, mấtUSB, quyềnAUTO, tư thế và D18 hiện hữu vẫn được giữ.
- Firmware, thư việnArduino và `.env` giữ nguyên byte so vớiZIPV3.16.12; không thiếu tệp gốc. Chi tiết tại `integrity_v31613.json`.
- Chưa kết nối hoặc thử trênrobot thật. Mô phỏng không xác nhận vị trí cơ khí hay độ tin cậy cảm biến thực tế.
