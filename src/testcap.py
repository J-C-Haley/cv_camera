import cv2

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
# cap = cv2.VideoCapture(1,cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y', '1', '6', ' '))
cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

while(True):
    ret, frame = cap.read()
    if ret:
        cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()