# ROS2 Lifecycle 기반 Mission Control System

이 프로젝트는 **ROS2 Lifecycle Node**를 기반으로,
`core` 파일과 `missions` 파일을 분리하여 **미션 상태를 제어 및 관리**할 수 있도록 설계된 프로그램입니다.

`core` 노드는 각 mission의 lifecycle 상태를 제어하며,
각 `mission` 노드는 독립적인 lifecycle node로 동작합니다.

---

## 📌 주요 특징

* ROS2 **LifecycleNode** 구조 사용
* `core` 노드를 통해 mission 상태 중앙 제어
* CLI 명령어를 통해 **mission lifecycle 상태 직접 변경 가능**
* 미션별 독립 실행 구조
* 자율주행 시나리오(Autorace)에 맞춘 미션 구성

---

## 🗂 프로젝트 구조 개요

```text
autorace_py/
├── core/
│   └── core_node.py        # mission lifecycle 상태 제어
├── missions/
│   ├── traffic_light.py   # 신호등 감지
│   ├── sign_board.py      # 방향 표지판 감지
│   ├── barrier.py         # 차단바 감지
│   └── obstacle.py        # 장애물 감지
│   └── line_tracing.py    # 차선 추적 노드
├── launch/
│   └── missions_launch.py # missions 실행 launch 파일
```

---

## 🔄 Lifecycle Node 상태 제어

각 mission은 ROS2 Lifecycle Node로 구성되어 있으며,
아래 명령어를 통해 **직접 상태 변경**이 가능합니다.

```bash
ros2 lifecycle set /{mission_name} {status}
```

### 예시

```bash
ros2 lifecycle set /line_tracing configure
ros2 lifecycle set /traffic_light activate
ros2 lifecycle set /barrier deactivate
```

---

## 🚀 실행 방법

### 1️⃣ Missions 실행 (감지 미션들)

아래 launch 파일을 통해 다음 미션들을 실행할 수 있습니다.

* 방향 표지판 감지
* 신호등 감지
* 차단바 감지
* 장애물 감지

```bash
ros2 launch autorace_py missions_launch.py
```

---

### 2️⃣ Line Tracing 실행

Line tracing 노드 또한 별도로 실행해야 합니다.

```bash
ros2 run autorace_py line_tracing
```

---

### 3️⃣ Core Node 실행

Mission 상태를 제어하는 `core` 노드는 **단독 실행**합니다.

```bash
ros2 run autorace_py core
```

---

## ⚙️ 시스템 동작 흐름

1. `missions_launch.py`로 감지 미션 노드 실행
2. `core` 노드 실행 후 mission lifecycle 상태 관리
3. 필요 시 CLI 명령어로 mission 상태 수동 변경
4. `line_tracing` 노드는 독립적으로 주행 제어 수행

---

## 🧩 활용 예시

* 특정 미션만 활성화 / 비활성화
* 주행 중 상황에 따라 mission 전환
* 테스트 및 디버깅 시 lifecycle 상태 개별 제어

---
