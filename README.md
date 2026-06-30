# 🛩️ Aircraft Attitude Control with LQR
 
행렬 기반 상태 공간 모델과 LQR(Linear Quadratic Regulator) 제어를 활용한  
항공기 자세(Attitude) 제어 시뮬레이터입니다.
 
---
 
## 📌 프로젝트 개요
 
| 항목 | 내용 |
|------|------|
| 목표 | 항공기의 피치·요·롤 축 자세를 최적 제어로 안정화 |
| 제어 기법 | LQR (Linear Quadratic Regulator) |
| 수치 적분 | 오일러 전진법 (Euler Forward Method) |
| 언어 | Python 3 |
| 주요 라이브러리 | `numpy`, `scipy` |
 
---
 
## 🧮 제어 이론 개요
 
### 상태 공간 모델
 
시스템은 6차원 상태 벡터로 표현됩니다:
 
```
x = [pitch, pitch_rate, yaw, yaw_rate, roll, roll_rate]ᵀ
```
 
상태 방정식:
 
```
ẋ = Ax + Bu
y = Cx + Du
```
 
- `A` : 6×6 시스템 행렬
- `B` : 6×3 입력 행렬 (피치·요·롤 각각 독립 제어 입력)
- `C` : 6×6 단위 행렬 (전체 상태 관측)
- `D` : 6×3 영행렬
### LQR 제어기 설계
 
연속시간 리카티 방정식(CARE)을 풀어 최적 제어 이득 `K`를 계산합니다:
 
```
AᵀX + XA - XBR⁻¹BᵀX + Q = 0
K = R⁻¹BᵀX
```
 
피드백 제어 법칙:
 
```
u = -Kx
```
 
가중 행렬은 단위 행렬로 설정 (`Q = I₆`, `R = I₃`).
 
---
 
## 🚀 실행 방법
 
### 설치
 
```bash
pip install numpy scipy
```
 
### 실행
 
```bash
python Aircraft_Attitude_Control.py
```
 
### 출력 예시
 
```
Initial State: [5. 0. 3. 0. 2. 0.]
Final State after simulation: [근사 0에 수렴하는 값]
```
 
초기 자세 오차(피치 5°, 요 3°, 롤 2°)가 LQR 제어에 의해 0으로 수렴합니다.
 
---
 
## 📁 파일 구조
 
```
Aircraft_Attitude_Control.py   # 상태 공간 모델, LQR 설계, 시뮬레이션 전체 포함
```
 
---
 
## 📝 회고
 
제어공학의 핵심 개념인 상태 공간 표현과 LQR을 직접 구현해본 프로젝트입니다.  
행렬 연산만으로 다축(피치·요·롤) 자세를 동시에 안정화할 수 있다는 점이  
선형 제어 이론의 강력함을 체감하게 해주었습니다.
