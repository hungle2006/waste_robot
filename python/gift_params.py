"""Servo phát quà D4. Sửa rồi lưu; lần bấm Đổi quà tiếp theo đọc lại tệp này."""

# Hai vị trí có thể chỉnh, đơn vị microsecond (us), mỗi giá trị 500..2500.
# Chiều mở/đóng phụ thuộc cách lắp cơ khí; hiệu chỉnh trước khi dùng với quà.
GIFT_OPEN_US = 600
GIFT_CLOSE_US = 2500  # Vị trí ban đầu và vị trí trả về sau mỗi lần nhấn.

# Thời gian giữ ở vị trí mở trước khi tự trả về vị trí đóng, millisecond.
GIFT_OPEN_HOLD_MS = 800

# Chờ servo về vị trí đóng trước khi cho phép lần phát tiếp theo.
GIFT_CLOSE_SETTLE_MS = 500
