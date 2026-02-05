# SLAM이란 무엇인가? (Simultaneous Localization and Mapping)

> 함께 읽기: [Nav2](what_is_nav2.md), [AMCL](what_is_amcl.md)

## 한 줄 요약
**SLAM**은 로봇이 **(1) 내가 어디 있는지(Localization)**와 **(2) 주변 지도가 어떻게 생겼는지(Mapping)**를
**동시에** 추정하는 문제/기술의 총칭입니다.

![SLAM 개념 이미지](img/SLAM_Visualisierung_01.png)

---

## 왜 SLAM이 “어렵다”는 말을 많이 하나?
SLAM이 어려운 이유는 한 문장으로 정리됩니다.

- 지도를 만들려면 로봇의 위치를 알아야 하고,
- 위치를 알려면 지도가 필요합니다.

즉, **닭-달걀(상호의존)** 구조를 가지며, 센서 노이즈/오도메트리 드리프트/환경 변화까지 겹치면 오차가 누적되기 쉽습니다.

---

## SLAM을 구성하는 핵심 아이디어(큰 그림)
SLAM 알고리즘마다 구현은 다르지만, 보통 아래 덩어리로 나눠 생각하면 정리가 쉽습니다.

### 1) 프런트엔드(Front-end): “지금 센서가 본 게 무엇과 대응되나?”
- 라이다 스캔 매칭(scan matching), 특징점 매칭, 데이터 연관(data association)
- 인접한 순간 사이의 상대 이동(odometry 보정 포함)을 추정

### 2) 백엔드(Back-end): “전체 궤적이 가장 그럴듯하게 되도록”
- 포즈 그래프(pose graph)를 만들고, 루프 클로저(loop closure)를 넣어
- 그래프 최적화(least squares 등)로 누적 오차를 줄임

### 3) 맵 표현(Map representation)
- 2D 점유격자(Occupancy Grid), 3D 포인트클라우드/서펠(surfel) 등
- Nav2에서는 대표적으로 2D Occupancy Grid(`/map`)가 많이 쓰입니다.

---

## Localization(AMCL)과 SLAM의 차이
둘 다 “내가 어디인지”를 추정하지만, 전제가 다릅니다.

- **AMCL**: “맵이 이미 있다” → 그 맵 위에서 위치만 찾는다  
- **SLAM**: “맵이 없다(또는 신뢰할 수 없다)” → 위치와 맵을 함께 만든다

실무에서는 흔히 아래 흐름을 많이 씁니다.
1) SLAM으로 맵 생성(한 번 또는 가끔)  
2) 그 맵을 고정해서 AMCL로 로컬라이제이션 + Nav2 운영

---

## ROS 2에서 SLAM을 볼 때 꼭 보는 것들
SLAM이 “잘 되고 있다/안 되고 있다”는 보통 아래 신호에서 드러납니다.

### 1) `/map`이 정상적으로 갱신되는가?
- RViz에서 Map 디스플레이가 찢어지지 않고(왜곡), 벽이 제자리에 쌓이는지 확인합니다.

### 2) TF가 올바른가? (`map → odom` 제공 여부)
- 많은 SLAM 노드는 **`map → odom`** 변환을 계산해서 브로드캐스트합니다.
- 오도메트리는 `odom → base_link`를 제공합니다.
- 결국 `map → base_link`가 이어져야 Nav2도 안정적으로 동작할 수 있습니다.

### 3) 입력(특히 라이다) 프레임/시간이 일치하는가?
- `/scan`의 `frame_id`가 TF 트리에 존재하는지
- 시뮬레이션이면 `use_sim_time:=True`를 켰는지(시간 불일치면 필터가 메시지를 버리기 쉽습니다)

---

## slam_toolbox 관점에서 보는 SLAM (이 저장소에서 사용)
**slam_toolbox**는 ROS 2에서 많이 쓰는 **2D 라이다 기반 SLAM** 패키지입니다.

- Online SLAM(실시간 맵 생성)과 Offline 처리(로그 기반) 모두 지원
- 맵 저장/불러오기(serialize), 루프 클로저 등 실전 기능을 다수 포함

이 저장소의 실습 흐름:
- 지도 생성: `README.md`의 “2) Online SLAM”
- 저장한 맵으로 Nav2 운용: `README.md`의 “3) (선택) 저장한 맵으로 다시 Nav2 주행하기”

---

## 자주 겪는 문제(체크리스트)
- **맵이 지그재그/늘어남**: 오도메트리 품질 저하(슬립), 스캔 매칭 실패, 라이다 각도/프레임 문제 가능  
- **루프 클로저 후 갑자기 맵이 튐**: 루프 클로저는 오차를 줄이지만, 순간적으로 프레임이 재정렬되며 “점프”처럼 보일 수 있음  
- **시간 불일치**: 시뮬레이션에서 `use_sim_time:=True` 누락 시 자주 발생  
- **TF 끊김**: `map → odom → base_link`가 이어지는지부터 확인

---

## 참고 자료(우선순위)
- SLAM 개요(위키): https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping
- slam_toolbox ROS Wiki: https://wiki.ros.org/slam_toolbox
- slam_toolbox GitHub: https://github.com/SteveMacenski/slam_toolbox
- Cartographer(다른 SLAM 예시) 문서: https://google-cartographer.readthedocs.io/en/latest/
- 이해하기 쉬운 영상(입문용): https://www.youtube.com/watch?v=mzeWl18PrUQ
- 교과서(권장): *Probabilistic Robotics* (Thrun, Burgard, Fox)
