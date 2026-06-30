
import numpy as np
import scipy.linalg 

class AircraftAttitudeControl:
     def __init__(self):
         # 상태 공간 모델의 매개변수 임의로 설정
         self.A = np.array([
             [0, 1, 0, 0, 0, 0],
             [0, 0, -0.1, 0, 0, 0],
             [0, 0, 0, 1, 0, 0],
             [0, 0, 0.1, 0, 0, 0],
             [0, 0, 0, 0, 0, 1],
             [0, 0, 0, 0, 0, -0.1]
         ])
         self.B = np.array([
             [0, 0, 0],
             [1, 0, 0],
             [0, 0, 0],
             [0, 1, 0],
             [0, 0, 0],
             [0, 0, 1]
         ])
         self.C = np.eye(6)
         self.D = np.zeros((6, 3))
         
         # 제어 이득 행렬 K 계산 (LQR 방법 사용하여 계산)
         Q = np.eye(6)
         R = np.eye(3)
         self.K = self.lqr(self.A, self.B, Q, R)
     
     def lqr(self, A, B, Q, R):
         # 리카티 방정식 풀이
         X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
         
         # LQR gain 계산
         K = np.matrix(scipy.linalg.inv(R) * (B.T * X))
         
         return np.asarray(K)
    
     def control(self, state):
         # 피드백 제어 계산
         u = -np.dot(self.K, state)
         return u
     
     def simulate(self, initial_state, steps=100, dt=0.01):
         state = initial_state
         states = [state]
         for _ in range(steps):
             u = self.control(state)
             state_dot = np.dot(self.A, state) + np.dot(self.B, u)
             state = state + state_dot * dt
             states.append(state)
         return np.array(states)
 
# 초기 상태 (피치, 요, 롤 축 각도와 각속도)
initial_state = np.array([5.0, 0, 3.0, 0, 2.0, 0])  # 각도는 ‘도‘ 단위
 
# 시스템 생성 및 시뮬레이션
aircraft_control = AircraftAttitudeControl()
states = aircraft_control.simulate(initial_state)
 
# 결과 출력
print("Initial State:", initial_state)
print("Final State after simulation:", states[-1])