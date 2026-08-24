"""GEOM — 외형 · 배치 · 내장 판정.  ICD0-008 §5.1

함수 세 개의 실행 시점이 다르다: hull 은 ⓪, layout·check_fit 은 ②.

hull  — [구조 완료 · 검증 완료] 대수 계산만, 반복 없음. 손계산과 대조했다 (__main__).
layout·check_fit — [스텁] 사이징 후 확정된 부품 치수가 있어야 해서 P6 대상이다.
"""
import math

import constants as k
from interfaces import DesignVars, HullOut, LayoutOut, FitOut


class GeomInfeasible(ValueError):
    """⓪ 형상 성립 조건 위반 — 이 설계점은 애초에 계산이 안 된다.

    ICD §6.1 의 g 목록에는 ⓪ 형상 판정이 없다(g1 은 프롭 항목이다). 그런데 형상이
    성립하지 않는 조합은 실제로 존재하고, 지금은 음수 길이나 동체 밖 핀이 조용히
    뒤로 흘러간다. 런처가 이것을 받아 사유 코드로 탈락시킨다 — g 를 새로 만들지
    않으므로 §6.1 은 그대로다. [확정 필요 — ICD 등재 요청]
    """


def hull(dv: DesignVars) -> HullOut:
    """⓪ 외형 확정 — 대수 계산만, 반복 없음.

    무게·부품과 무관하다. ICD0-008 은 포드(모터 나셀)를 hull 출력에서 뺐다 —
    포드 치수는 모터 질량의 함수이고 모터는 ① 사이징 결과라 ⓪에서 알 수 없다.

    형상이 성립하지 않으면 GeomInfeasible 을 던진다.
    """
    # ── 동체 ──
    r_body = dv.d_body / 2.0
    S_ref = math.pi * dv.d_body ** 2 / 4.0      # 기준면적 = 동체 최대 단면적
    l_body = dv.d_body * dv.lambda_body
    l_nose = k.f_nose * dv.d_body
    l_cyl = l_body - l_nose

    # ── 핀 제원 역산 (S_fin 은 4매 총면적, AR_fin = b²/S 편측 기준) ──
    S_1 = dv.S_fin / k.N_rot                    # 1매 노출 면적
    b_fin = math.sqrt(dv.AR_fin * S_1)          # AR = b²/S 정의에서
    c_root = 2.0 * S_1 / (b_fin * (1.0 + k.lam_fin))   # 사다리꼴 면적 = b(c_r+c_t)/2
    c_tip = k.lam_fin * c_root
    x_t = c_root - c_tip                        # 앞전 스윕 — 뒷전을 정렬시킨 전제
    t_fin = k.tc_fin * c_root

    # ── ⓪ 형상 성립 조건 ──
    if l_cyl <= 0.0:
        # lambda_body < f_nose 면 노즈가 전장보다 길다.
        raise GeomInfeasible(
            f"원통부 길이가 {l_cyl:+.4f} m — lambda_body({dv.lambda_body}) 가 "
            f"노즈 세장비 f_nose({k.f_nose}) 이하다")
    if dv.x_fin + c_root > l_body:
        # 핀 뒷전이 동체 뒤로 튀어나간다. 이걸 통과시키면 x_cp 가 전장 밖에 찍히고
        # (x_cp/l_body > 1) STAB 의 안정여유가 물리적 의미를 잃는다.
        raise GeomInfeasible(
            f"핀 뒷전 {dv.x_fin + c_root:.4f} m 가 전장 {l_body:.4f} m 를 넘는다 "
            f"(x_fin={dv.x_fin}, c_root={c_root:.4f})")
    if dv.x_fin < l_nose:
        # 핀 앞전이 노즈 원추부에 물린다 — Barrowman 의 핀-동체 간섭 전제가 깨진다.
        raise GeomInfeasible(
            f"핀 앞전 {dv.x_fin} m 가 노즈 끝 {l_nose:.4f} m 보다 앞이다")

    # ── 젖음면적 ── (ICD §5.1: 노즈 표면적 + 원통 측면적 + 핀 양면)
    S_wet_nose = k.k_nose * math.pi * dv.d_body * l_nose
    S_wet_cyl = math.pi * dv.d_body * l_cyl
    S_wet_fin = 2.0 * dv.S_fin * k.k_thk        # 양면 + 두께 할증
    S_wet_body = S_wet_nose + S_wet_cyl

    return HullOut(
        S_ref=S_ref, S_wet=S_wet_body + S_wet_fin,
        l_body=l_body, r_body=r_body,
        b_fin=b_fin, c_root=c_root, c_tip=c_tip, t_fin=t_fin, x_t=x_t,
        x_fin=dv.x_fin, l_nose=l_nose, l_cyl=l_cyl,
        S_wet_body=S_wet_body, S_wet_fin=S_wet_fin,
    )


def layout(dv: DesignVars, hl: HullOut, dims: dict) -> LayoutOut:
    """② 부품 배치 — 사이징 후 확정된 치수를 받아 기수부터 쌓는다.

    dims : {부품명: (L, W, H)} — 배터리·모터는 부피 = 질량/rho_* 로 환산해 온다.

    [스텁] 구현 예정 (P6):
        배치 규칙 순서 — 카메라 → 센서 → 배터리 → FC/ESC
        arm_rotor = r_body + f_mount · b_fin (포드가 핀에 결합)
    """
    return LayoutOut(
        x_parts={name: 0.0 for name in dims},   # [스텁] 전부 기수에 겹쳐 둔 상태
        arm_rotor=1.0,                          # [스텁] f_mount 에 반응하지 않는다
    )


def check_fit(dv: DesignVars, hl: HullOut, lay: LayoutOut, dims: dict) -> FitOut:
    """② 내장 · 클리어런스 판정. 둘 다 양수면 합격.

    [스텁] 구현 예정 (P6):
        g6 = 가용 길이 − 부품 점유 길이 (단면 여유도 함께 검사)
        g7 = arm_rotor − d_prop/2 − r_body − d_clr
    """
    return FitOut(
        g6=0.0,   # [스텁] 실제 판정 아님
        g7=0.0,   # [스텁]
    )


if __name__ == "__main__":   # 검산 — 손계산 대조 (TASKS P2 완료판정)
    from common.out import stdout_utf8
    stdout_utf8()

    dv = DesignVars(d_body=0.09, lambda_body=7.0, S_fin=0.036, x_fin=0.50,
                    AR_fin=2.2, f_mount=0.8, n_design=4.0, d_prop=0.13,
                    pd_prop=1.30, n_ser=6, k_E=1.0, k_mot=1.0)
    h = hull(dv)

    # 손계산: S_ref = π(0.09)²/4,  l_body = 0.09×7
    assert abs(h.S_ref - math.pi * 0.09 ** 2 / 4) < 1e-15, h.S_ref
    assert abs(h.l_body - 0.63) < 1e-15, h.l_body
    # 핀 역산이 원래 면적을 복원하는가 — b(c_r+c_t)/2 × 4 == S_fin
    S_back = k.N_rot * h.b_fin * (h.c_root + h.c_tip) / 2.0
    assert abs(S_back - dv.S_fin) < 1e-15, (S_back, dv.S_fin)
    # AR 정의 복원
    assert abs(h.b_fin ** 2 / (dv.S_fin / k.N_rot) - dv.AR_fin) < 1e-12
    # 젖음면적 합 항등
    assert abs(h.S_wet - (h.S_wet_body + h.S_wet_fin)) < 1e-15

    # ⓪ 형상 성립 조건이 실제로 걸리는가
    import dataclasses
    for label, over in [("노즈 > 전장", {"lambda_body": 2.5}),
                        ("핀이 동체 뒤", {"lambda_body": 5.0}),
                        ("핀이 노즈에 물림", {"x_fin": 0.20})]:
        try:
            hull(dataclasses.replace(dv, **over))
            raise AssertionError(f"{label}: 걸러졌어야 한다")
        except GeomInfeasible as e:
            print(f"[탈락] {label:<14} → {e}")

    print(f"S_ref      = {h.S_ref:.6e} m²   (손계산 {math.pi * 0.09**2/4:.6e})")
    print(f"l_body     = {h.l_body:.6f} m    (손계산 {0.09*7:.6f})")
    print(f"l_nose/cyl = {h.l_nose:.6f} / {h.l_cyl:.6f} m")
    print(f"b_fin      = {h.b_fin:.6f} m    c_root={h.c_root:.6f}  c_tip={h.c_tip:.6f}")
    print(f"S_fin 복원 = {S_back:.9f} m²  (입력 {dv.S_fin})")
    print(f"S_wet      = {h.S_wet:.6f} m²  (동체 {h.S_wet_body:.6f} + 핀 {h.S_wet_fin:.6f})")
    print("GEOM.hull 검산 통과")
