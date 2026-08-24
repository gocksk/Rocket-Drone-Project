"""AERO — 항력 빌드업 · 안정성 계수.  ICD0-008 §5.1

종속성: 무게·부품과 무관 → ⓪에서 1회만 실행하고, 반복 안에서는 곡선을 조회만 한다.

[스텁] P2 에서 구현한다. 반환값은 자리표시이며 물리적 근거가 없다.
       CL(α) 는 이 판에서 추가된 항목이고 트림 연립(§4.1)의 전제다 —
       항력만 내고 CL 을 0 으로 두면 P3 완료판정("요구추력 < √(D²+W²)")이 성립하지 않는다.

[결정 필요] α 와 자세각 θ 의 관계 정의 — 순항 비행경로각 가정. P3 에서 확정한다 (ICD §8 A-1).
"""
import constants as k
from interfaces import DesignVars, HullOut, AtmOut, AeroOut


def run(dv: DesignVars, hl: HullOut, air: AtmOut) -> AeroOut:
    """⓪ 항력 빌드업 + Barrowman.

    [스텁] 구현 예정 (P2):
        성분별 항력 빌드업 — 동체 마찰(Re 기반 평판 상관식 × 형상계수)
                             + 핀 마찰·압력 + 기저 항력 + 간섭 항력
        압축성 보정 — a_snd 대비 마하수로
        법선력·압력중심 — Barrowman (노즈 기여 + 핀 기여)
    """
    q_cr = 0.5 * air.rho * k.V_cr ** 2      # 동압 정의 — 물리 모델이 아니라 정의식

    def F_drag(V: float, alpha: float = 0.0) -> float:
        """(V, α) → 항력 [N].  [스텁] 실제 항력 아님."""
        return 0.0          # [스텁]

    def CL(alpha: float) -> float:
        """α → 양력계수 (S_ref 기준).  [스텁] 실제 값 아님 — 트림 연립의 전제."""
        return 0.0          # [스텁]

    return AeroOut(
        F_drag=F_drag,
        CL=CL,
        CN_alpha=0.0,       # [스텁]
        x_cp=0.0,           # [스텁]
        CN_alpha_fin=0.0,   # [스텁]
        q_cr=q_cr,
        CD0_cr=0.0,         # [스텁]
    )
