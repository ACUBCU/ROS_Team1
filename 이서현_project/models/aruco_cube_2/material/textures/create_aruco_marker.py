import cv2

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

marker_id = 1
image_size = 1000

marker = cv2.aruco.drawMarker(
    dictionary,
    marker_id,
    image_size,
)

# 흰색 여백을 추가하면 검출이 더 안정적이다.
marker_with_margin = cv2.copyMakeBorder(
    marker,
    100,
    100,
    100,
    100,
    cv2.BORDER_CONSTANT,
    value=255,
)

cv2.imwrite("aruco_1.png", marker_with_margin)
print("aruco_1.png 생성 완료")
