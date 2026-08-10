# 지금 저(Claude)에게 도와주실 수 있는 것들

이 문서는 **지금 이 순간 막혀 있거나, 제가 절대 혼자 할 수 없는 것들만** 정리한 겁니다. 프로젝트 전체 설명은 [project_summary.md](project_summary.md) 등 다른 문서를 보세요. 여기 있는 걸 해보시고 결과(성공했는지, 어떤 에러/로그가 떴는지, 화면이 어떻게 보였는지)만 저한테 알려주시면 됩니다.

---

## 부탁 1 (선택) — RViz 아루코 시각화가 실제로 보기 좋은지 확인

제가 데이터(토픽 값)는 확인했지만, **화면에 예쁘게/이해하기 쉽게 보이는지는 확인 못 했습니다.** (참고: `web_control.py` 웹 페이지에서는 카메라 화면을 이미 직접 볼 수 있으니, 이건 RViz로 보고 싶을 때만 필요한 선택 사항입니다.)

RViz를 켜고(`ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py use_sim:=true start_rviz:=true`) "Add" → **Image** (토픽: `/aruco_detection/image`), **Marker** (토픽: `/aruco_detection/markers`) 추가해보시고, `box_sort_project.py`를 실행한 상태에서 카메라 앞에 아루코 박스가 있을 때:
- 이미지 화면에 초록색 테두리/좌표축이 잘 그려지는지
- 3D 공간에 "id=0 dist=0.15m" 같은 글자가 마커 위에 떠 있는지, 위치가 이상하지 않은지

여기는 이미 데이터상으로는 확인됐어서, 안 예뻐 보여도 급한 건 아닙니다.
