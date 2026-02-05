# Nav2란 무엇인가? (ROS 2 Navigation2)

> 함께 읽기: [SLAM](what_is_slam.md), [AMCL](what_is_amcl.md)

## 한 줄 요약
**Nav2**는 ROS 2에서 “로봇을 목표점까지 안전하게 이동”시키기 위한 **내비게이션 프레임워크**입니다.  
센서(라이다/카메라), 지도(맵), 로봇의 위치추정(AMCL/SLAM), 경로 계획, 장애물 회피, 복구 동작 등을
여러 노드와 플러그인 형태로 조립해서 사용합니다.

![Nav2 아키텍처](img/nav2_architecture.png)

---

## Nav2가 풀어주는 문제(무엇을 해주나?)
Nav2의 최종 목표는 단순합니다: **“A에서 B로 간다.”**  
하지만 현실에서는 아래 문제가 같이 따라옵니다.

- **로봇이 지금 어디에 있는가?** (Localization: AMCL/SLAM)
- **어떤 경로로 갈 것인가?** (Global planning)
- **주행 중 장애물을 어떻게 피할 것인가?** (Local planning / obstacle avoidance)
- **막혔을 때 어떻게 회복할 것인가?** (Recovery behaviors)
- **안정적으로 운영하려면?** (Lifecycle, parameter, diagnostics)

---

## Nav2의 구성요소(큰 덩어리)
배포판과 설정에 따라 구성은 달라질 수 있지만, 보통 아래 구성으로 이해하면 가장 안전합니다.

### 1) “결정과 순서”: BT Navigator
- `bt_navigator`는 **Behavior Tree(BT)**로 “언제 플래너를 호출하고, 언제 컨트롤러를 돌리고, 언제 복구할지”를 결정합니다.
- 사용자는 보통 RViz에서 **Nav2 Goal**을 찍거나, 액션으로 `NavigateToPose`를 호출합니다.

### 2) “큰 길”: Planner Server (Global Planner)
- `planner_server`는 지도/코스트맵을 기반으로 **전역 경로(global path)**를 계산합니다.
- 내부는 **플러그인**으로 바뀔 수 있습니다(예: Grid 기반, Hybrid-A*, Smac 계열 등).

### 3) “당장 바퀴를 돌리는 법”: Controller Server (Local Controller)
- `controller_server`는 전역 경로를 따라가도록 **속도 명령(`/cmd_vel`)**을 생성합니다.
- 센서 기반 장애물 회피도 여기서 강하게 영향을 받습니다.

### 4) “안전하게 보기”: Costmaps (Global / Local)
- Nav2는 보통 **2개의 코스트맵**을 둡니다.
  - Global costmap: 멀리 보는 지도 기반(Static layer 중심)
  - Local costmap: 주변 장애물 반영(Obstacle layer 중심)
- 레이어(대표 예시): static / obstacle(voxel) / inflation

### 5) “운영”: Lifecycle Manager
- Nav2의 여러 노드는 **Lifecycle node**로 관리됩니다.
- `lifecycle_manager`가 노드들을 `configure → activate` 순으로 올리고, 문제 시 상태를 관리합니다.

---

## 동작 흐름(Goal을 찍으면 내부에서 무슨 일이 일어나나?)
아래 순서로 “대체로” 흘러간다고 생각하면 좋습니다.

1) 사용자가 Goal을 보냄(RViz 버튼 또는 액션 호출)  
2) `bt_navigator`가 플래닝/컨트롤/복구의 순서를 BT로 실행  
3) `planner_server`가 global path 생성  
4) `controller_server`가 주기적으로 `/cmd_vel` 생성(로봇 주행)  
5) 센서/코스트맵 업데이트로 경로 재계획 또는 회피  
6) 막히면 recovery(회전/후진/코스트맵 클리어 등)  
7) Goal 도착 시 성공 반환

---

## TF(좌표계)가 흔들리면 Nav2는 거의 무조건 실패한다
Nav2에서 가장 자주 발생하는 문제는 “플러그인”이 아니라 **TF/프레임**입니다.

### 기본적으로 기대하는 프레임
- `map` : 전역 기준 좌표(지도 좌표)
- `odom` : 짧은 시간 안정적인 좌표(바퀴/IMU 적분 기반, 드리프트 가능)
- `base_link` (또는 `base_footprint`) : 로봇 본체 기준

### 꼭 성립해야 하는 TF 관계(핵심)
- **Localization/SLAM**이 `map → odom`을 제공
- **Odometry**가 `odom → base_link`를 제공

즉, 결국 `map → base_link`가 이어져야 Nav2가 “로봇이 어디 있는지” 알 수 있습니다.

확인 명령(선택):
```bash
ros2 run tf2_tools view_frames
```

---

## 실습과 연결(이 저장소에서 어디를 보면 되나?)
- Map-based Navigation: `README.md`의 “1) Map-based Navigation”
- Online SLAM: `README.md`의 “2) Online SLAM”

---

## 체크리스트(안 될 때 먼저 보는 순서)
1) 모든 터미널에서 `source /opt/ros/humble/setup.bash` 했는가?  
2) 시뮬레이션이면 `use_sim_time:=True`를 **Nav2 / SLAM 모두**에 줬는가?  
3) `map → odom → base_link` TF가 이어지는가?  
4) `/scan`(라이다)과 `/cmd_vel`(속도명령)이 정상인지 확인했는가?  
5) RViz에서 초기 위치(**2D Pose Estimate**)를 찍었는가? (AMCL 사용 시 특히 중요)

---

## 참고 자료(우선순위)
- Nav2 공식 문서: https://docs.nav2.org/
- Nav2 GitHub: https://github.com/ros-navigation/navigation2
- ROS 2 Launch / TF2 개념: https://docs.ros.org/en/humble/
- BehaviorTree.CPP(참고): https://www.behaviortree.dev/
