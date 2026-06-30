"""
항공기 자세 제어 시스템 (LQR 기반 오토파일럿)
Aircraft Attitude Control via Linear Quadratic Regulator

상태 벡터 x = [pitch, pitch_rate, roll, roll_rate, yaw, yaw_rate]  (단위: rad, rad/s)
입력 벡터 u = [delta_elevator, delta_aileron, delta_rudder]         (단위: rad)
"""

import math
import numpy as np
import scipy.linalg


class AircraftAttitudeControl:
    def __init__(self, wn=(2.0, 1.5, 1.0), zeta=(0.7, 0.7, 0.5),
                 Q_diag=(10, 1, 10, 1, 10, 1), R_diag=(1, 1, 1)):
        """
        Parameters
        ----------
        wn   : (wn_pitch, wn_roll, wn_yaw)  각 축 비감쇠 고유진동수 [rad/s]
        zeta : (z_pitch,  z_roll,  z_yaw)   각 축 감쇠비 (0 < ζ < 1 : 부족감쇠)
        Q_diag : LQR 상태 가중치 대각원소
        R_diag : LQR 제어입력 가중치 대각원소
        """
        wn_p, wn_r, wn_y = wn
        z_p,  z_r,  z_y  = zeta

        # ── 상태 행렬 A: 2차 선형 모델 (축 간 비결합 가정) ──────────────
        # 각 축: [θ̇ = ω,  ω̇ = -wn²·θ - 2ζwn·ω]
        # 원본 코드의 A[1,2] = -0.1, A[3,2] = +0.1 은
        #   피치율 변화율이 롤각에, 롤율 변화율이 롤각 자체에 잘못 결합되어
        #   롤 서브시스템에 불안정 극(eigenvalue = +0.316)을 만들었음.
        self.A = np.array([
            [0,          1,           0,           0,          0,           0         ],
            [-wn_p**2,  -2*z_p*wn_p,  0,           0,          0,           0         ],
            [0,          0,           0,           1,          0,           0         ],
            [0,          0,          -wn_r**2,    -2*z_r*wn_r, 0,           0         ],
            [0,          0,           0,           0,          0,           1         ],
            [0,          0,           0,           0,         -wn_y**2,    -2*z_y*wn_y],
        ])  # (6×6)

        # ── 입력 행렬 B: 각 조종면이 각속도에만 직접 작용 ────────────────
        self.B = np.array([
            [0, 0, 0],
            [1, 0, 0],   # 엘리베이터 → 피치율
            [0, 0, 0],
            [0, 1, 0],   # 에일러론  → 롤율
            [0, 0, 0],
            [0, 0, 1],   # 러더      → 요율
        ])  # (6×3)

        # ── 제어가능성 검사 (LQR 적용 전 필수) ──────────────────────────
        if not self._is_controllable(self.A, self.B):
            raise ValueError("시스템이 완전 제어 불가능합니다. A, B 행렬을 확인하세요.")

        # ── LQR 이득 행렬 K 계산 ─────────────────────────────────────────
        Q = np.diag(Q_diag)   # 상태 편차 페널티
        R = np.diag(R_diag)   # 제어 에너지 페널티
        self.K = self._lqr(self.A, self.B, Q, R)

        # ── 폐루프 안정성 검증 ──────────────────────────────────────────
        A_cl = self.A - self.B @ self.K
        cl_eigs = np.linalg.eigvals(A_cl)
        if any(np.real(cl_eigs) >= 0):
            raise RuntimeError(f"폐루프 시스템이 불안정합니다. 고유값: {cl_eigs}")

    # ── 내부 유틸리티 ────────────────────────────────────────────────────

    @staticmethod
    def _is_controllable(A, B):
        """제어가능성 행렬의 랭크로 완전 제어가능성 검사"""
        n = A.shape[0]
        C = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)])
        return np.linalg.matrix_rank(C) == n

    @staticmethod
    def _lqr(A, B, Q, R):
        """
        연속 리카티 방정식 P를 풀고 LQR 이득 K = R⁻¹ BᵀP 반환.

        변경점:
        - np.matrix(…) 완전 제거 (NumPy 1.25+ 에서 deprecated)
        - inv(R) * (B.T * X) → np.linalg.solve(R, B.T @ P)
          (역행렬 명시 계산보다 수치적으로 더 안정적)
        """
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)  # ndarray (6×6)
        K = np.linalg.solve(R, B.T @ P)                    # ndarray (3×6)
        return K

    # ── 공개 API ─────────────────────────────────────────────────────────

    def control(self, state: np.ndarray) -> np.ndarray:
        """상태 피드백 제어입력 u = -Kx 계산"""
        return -self.K @ state

    def simulate(self, initial_state_deg: np.ndarray,
                 duration: float = 10.0, dt: float = 0.01) -> np.ndarray:
        """
        오일러 적분으로 닫힌 루프 응답 시뮬레이션.

        Parameters
        ----------
        initial_state_deg : [pitch°, pitch_rate°/s, roll°, roll_rate°/s, yaw°, yaw_rate°/s]
        duration          : 시뮬레이션 시간 [s]
        dt                : 적분 스텝 [s]

        Returns
        -------
        states_deg : (N+1, 6) 배열, 각도 열은 도(°), 각속도 열은 °/s
        """
        # 도 → 라디안 변환 (각도 인덱스 0,2,4 / 각속도 인덱스 1,3,5)
        state = initial_state_deg.astype(float).copy()
        state[0::2] = np.radians(state[0::2])   # 각도
        state[1::2] = np.radians(state[1::2])   # 각속도

        steps = int(duration / dt)
        states_rad = np.empty((steps + 1, 6))
        states_rad[0] = state

        for i in range(steps):
            u = self.control(state)
            state_dot = self.A @ state + self.B @ u
            state = state + state_dot * dt
            states_rad[i + 1] = state

        # 라디안 → 도 변환 후 반환
        states_deg = states_rad.copy()
        states_deg[:, 0::2] = np.degrees(states_rad[:, 0::2])
        states_deg[:, 1::2] = np.degrees(states_rad[:, 1::2])
        return states_deg


# ── 실행 예시 ────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # 초기 자세: 피치 5°, 롤 3°, 요 2° (각속도 0)
    initial_state = np.array([5.0, 0.0, 3.0, 0.0, 2.0, 0.0])

    controller = AircraftAttitudeControl()
    states = controller.simulate(initial_state, duration=10.0, dt=0.01)

    # 수렴 스텝 탐색 (0.1° 이내)
    threshold = 0.1
    converge_step = None
    for i, s in enumerate(states):
        if all(abs(s[0::2]) < threshold) and all(abs(s[1::2]) < threshold):
            converge_step = i
            break

    print("=" * 50)
    print(f"초기 상태 (도): {initial_state}")
    print(f"최종 상태 (도, t={len(states)*0.01-0.01:.1f}s): "
          f"{states[-1].round(4)}")
    if converge_step:
        print(f"0.1° 이내 수렴 시간: {converge_step * 0.01:.2f}s "
              f"({converge_step}스텝)")
    else:
        print("시뮬레이션 종료 후에도 미수렴")
    print("=" * 50)