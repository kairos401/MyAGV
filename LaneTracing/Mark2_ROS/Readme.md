# Mark2 ROS

### Cat CMD Threshold
- 기존의 Mark1 Simple Python의 알고리즘 사용
- 기존 알고리즘을 사용하여 ROS 기반 제어가 가능한지 확인하는 코드
- RQT를 이용하여 실시간 모니터링이 가능한지 확인

### Cat CMD Threshold Find
- Cat CMD Threshold에 라인을 놓쳤을 때 라인을 다시 찾는 코드 추가

### Cat CMD Sim
- 기존 알고리즘 (정지 후 각도 조절 후 출발)에서 실시간 각도를 통한 모터제어 로직으로 변경
- AGV가 정지하여 라인을 Tracing하는게 아닌 움직이며 실시간으로 각도를 이용하여 cmd_vel의 angular_z를 계산

### 확인할 부분
- 폭주? : 왜 아루코를 할 때는 폭주하는데 여기선 안하냐...