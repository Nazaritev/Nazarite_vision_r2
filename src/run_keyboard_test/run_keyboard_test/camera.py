import cv2

# 打开摄像头，0 表示 /dev/video0，1 表示 /dev/video1
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("读取帧失败")
        break

    cv2.imshow("Camera", frame)

    # 按 q 退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()