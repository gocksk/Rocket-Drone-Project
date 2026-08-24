"""GEOM — 외형 · 배치 · 내장 판정.  ICD0-008 §5.1

함수 세 개의 실행 시점이 다르다: hull 은 ⓪, layout·check_fit 은 ②.

[스텁] 이 파일은 배관만 깔려 있다. P2(hull)·P6 이후(layout·check_fit)에서 구현한다.
       아래 반환값은 전부 파이프라인이 흐르게 하려고 넣은 자리표시 숫자이며
       물리적 근거가 없다. STUBS.md 참조.
"""
import constants as k
from interfaces import DesignVars, HullOut, LayoutOut, FitOut


def hull(dv: DesignVars) -> HullOut:
    """⓪ 외형 확정 — 대수 계산만, 반복 없음.

    [스텁] 구현 예정 (P2):
        S_ref  = π·d_body²/4
        l_body = d_body × lambda_body
        S_wet  = 노즈 표면적 + 원통 측면적 + 핀 양면
        핀 스팬·코드를 S_fin 과 AR_fin 에서 역산
    """
    return HullOut(
        S_ref=1.0,      # [스텁] 실제 값 아님
        S_wet=1.0,      # [스텁]
        l_body=1.0,     # [스텁] ← EC C8 이 지금 가짜라는 뜻
        r_body=1.0,     # [스텁]
        b_fin=1.0,      # [스텁]
        c_root=1.0,     # [스텁]
        c_tip=1.0,      # [스텁]
        x_fin=dv.x_fin,
        l_nose=1.0,     # [스텁]
        l_cyl=1.0,      # [스텁]
    )


def layout(dv: DesignVars, hl: HullOut, dims: dict) -> LayoutOut:
    """② 부품 배치 — 사이징 후 확정된 치수를 받아 기수부터 쌓는다.

    dims : {부품명: (L, W, H)} — 배터리·모터는 부피 = 질량/rho_* 로 환산해 온다.

    [스텁] 구현 예정:
        배치 규칙 순서 — 카메라 → 센서 → 배터리 → FC/ESC
        arm_rotor 는 f_mount 와 핀 스팬에서 나온다
    """
    return LayoutOut(
        x_parts={name: 0.0 for name in dims},   # [스텁] 전부 기수에 겹쳐 둔 상태
        arm_rotor=1.0,                          # [스텁]
    )


def check_fit(dv: DesignVars, hl: HullOut, lay: LayoutOut, dims: dict) -> FitOut:
    """② 내장 · 클리어런스 판정. 둘 다 양수면 합격.

    [스텁] 구현 예정:
        g6 = 가용 길이 − 부품 점유 길이 (단면 여유도 함께 검사)
        g7 = arm_rotor − d_prop/2 − r_body − d_clr
    """
    return FitOut(
        g6=0.0,   # [스텁] 실제 판정 아님
        g7=0.0,   # [스텁]
    )
