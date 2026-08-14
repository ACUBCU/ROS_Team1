# 저작권 문제 없는 간단한 사람 실루엣(그림)을 코드로 그려서 PNG로 저장합니다.
# create_aruco_marker.py(aruco_cube 모델)와 같은 위치에 두는 자매 스크립트입니다.
#
# 실행 방법:
#   python3 create_person_silhouette.py
# person_silhouette.png가 이 폴더에 생성되면 model.sdf가 그걸 텍스처로 사용합니다.

from PIL import Image, ImageDraw

WIDTH, HEIGHT = 600, 1000
BG_COLOR = (255, 255, 255, 255)
FG_COLOR = (30, 30, 30, 255)

img = Image.new("RGBA", (WIDTH, HEIGHT), BG_COLOR)
draw = ImageDraw.Draw(img)

cx = WIDTH // 2

# 머리
head_r = 90
head_cy = 150
draw.ellipse(
    [cx - head_r, head_cy - head_r, cx + head_r, head_cy + head_r],
    fill=FG_COLOR,
)

# 목
draw.rectangle([cx - 25, head_cy + head_r - 10, cx + 25, head_cy + head_r + 40], fill=FG_COLOR)

# 몸통 (사다리꼴 - 어깨가 넓고 허리가 좁음)
shoulder_y = head_cy + head_r + 40
hip_y = shoulder_y + 380
draw.polygon(
    [
        (cx - 200, shoulder_y),
        (cx + 200, shoulder_y),
        (cx + 110, hip_y),
        (cx - 110, hip_y),
    ],
    fill=FG_COLOR,
)

# 팔 (양쪽, 몸통 옆에 붙은 두꺼운 선)
draw.rectangle([cx - 240, shoulder_y + 10, cx - 190, shoulder_y + 320], fill=FG_COLOR)
draw.rectangle([cx + 190, shoulder_y + 10, cx + 240, shoulder_y + 320], fill=FG_COLOR)

# 다리 (양쪽)
leg_top = hip_y
leg_bottom = HEIGHT - 40
draw.rectangle([cx - 100, leg_top, cx - 20, leg_bottom], fill=FG_COLOR)
draw.rectangle([cx + 20, leg_top, cx + 100, leg_bottom], fill=FG_COLOR)

img.convert("RGB").save("person_silhouette.png")
print("person_silhouette.png 생성 완료")
