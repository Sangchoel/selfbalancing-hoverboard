# 🛹 Self-Balancing Hoverboard

BNO055 IMU와 ODrive 모터 드라이버를 기반으로  
실시간 균형 제어가 가능한 **자이로 밸런싱 로봇(hoverboard)**를 구현한 프로젝트입니다.

Arduino 코드로 자세 제어 루프를 수행하며,  
Python 스크립트를 통해 ODrive 드라이버 설정 및 캘리브레이션을 자동화합니다.

---

## ⚙️ 시스템 구성도

```txt
BNO055 IMU         →   [Euler + Gyro]    →   Balance Controller (Arduino)
                                    ↓
ODrive (Motor 0,1)  ←   전류 제어 명령 (SetCurrent)
