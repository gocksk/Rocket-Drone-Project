"""입력 상수 = ICD0-007 §2의 코드판.

★ 표시는 잠정값이다. 「할 일 통합 목록」에서 확정되면 값만 바꾼다.
코드 안에 숫자를 직접 박지 말고 반드시 여기서 가져다 쓸 것.
"""
import math

# ── 공통 ──
N_rot = 4                 # 로터·핀·포드 수
g = 9.81                  # [m/s^2]

# ── 요구조건 ──
V_cr = 83.3               # 순항속도 300 km/h [m/s]

# ── 탑재 질량 ──
# m_avio (항전·센서 합 [kg])는 상수로 두지 않는다 — srl.avio() 목록의 합이 유일 출처이고
# WGHT가 거기서 직접 합산한다. 값을 여기 복제하면 목록이 바뀔 때 조용히 어긋난다. (ICD §2)

# ── 배터리 특성 (★같은 실제 팩의 짝이어야 함 — SRL §4) ──
DoD = 0.8
c_rate_max = 90.0         # ★ [1/h]
e_spec = 180.0            # ★ [Wh/kg]

# ── 합격 기준치 ──
tw_min = 1.6              # ★ g2 최소 호버 추력비
SM_min = 1.0              # g6 [cal]
k_ctrl = 1.5              # ★ g7 최소 조종 여유비
M_tip_max = 0.85          # ★ g5 팁 마하 한계 — 300 km/h급은 0.7로는 성립 불가 (스모크 테스트에서 확인)
R_dash_min = 3000.0       # ★ g8 최소 항속거리 [m]
k_w = 0.6                 # g10 핀 속 채움 폭 한계비

# ── 임무 정의 ──
h_to = 30.0               # ★ 이륙·착륙 고도 [m]
V_climb = 4.0             # ★ [m/s]
V_desc = 3.0              # ★ [m/s]
k_trans = 1.35            # ★ 천이 동력 계수
t_trans = 4.0             # ★ 천이 시간 [s]

# ── 여유·평가 규약 ──
eps_snap = 0.07           # ★ g1 버퍼 (5~10%)
# 소음 앵커 (★전부 실측으로 교체 예정 — 조건 세트)
SPL_ref = 78.0; r_ref = 3.0; V_tip_ref = 100.0; T_ref = 7.0; r_obs = 3.0
N_ref = 1                 # ★ 앵커의 로터 수 기준 — 미확인! 6 dB 걸린 문제

# ── 질량 계수 ──
k_wire = 0.10             # ★ 배선 할증 (PROP-A 단독 계상)
k_pack = 0.12             # ★ 배터리 하네스 계수

# ── 수렴 파라미터 ──
eps_conv = 1e-3
beta = 0.5
k_init = 1.4
N_iter_max = 50

# ══ 모듈별 계산 상수 (정의 원본은 각 가이드라인) ══

# GEOM (§9)
f_nose = 3.0; k_nose = 0.75
lam_fin = 0.5             # λ_fin 테이퍼비
tc_fin = 0.12             # ★ 핀 두께비 — STRC와 공동 확정
k_thk = 1.03; k_form = 1.05
t_pod = 0.002; f_pod = 3.5; f_pod_c = 0.4; d_hub = 0.010
t_0 = 0.0016; k_t = 0.0002; n_ref_load = 3.0   # ★ 벽두께 규칙 (n_ref)
d_end = 0.005             # δ_end
d_clr = 0.10              # δ_clr
d_ax = 0.010              # δ_ax

# AERO (§9)
k_base = 0.18             # ★
k_int = 0.10              # ★ 접합부 간섭
Cd_c = 1.2; eta_cf = 0.65 # 크로스플로
k_cp_nose = 0.466
k_cal = 1.0               # ★ 앵커로 결정
k_side = 0.67; k_xn = 0.6; k_finproj = 0.7

# PROP-A (§10)
eta_esc = 0.95
U_cell_nom = 3.7
k_T_cal = 1.0; k_Q_cal = 1.0   # ★ 앵커로 결정

# PROP-B (§8) — MISS와 공유
k_block = 0.95            # ★ 핀 블로케이지 (로터 후류가 핀에 막히는 손실) — PROP+GEOM 협의로 확정

# STRC (§10)
rho_mat = 1240.0          # ★ PLA [kg/m^3]
sigma_cat = 50e6          # ★ [Pa]
k_layer = 0.5; SF = 1.5
tau_allow = 8e6           # ★
phi_0 = 0.25; k_phi = 0.05     # 인필 규칙
n_peri = 3; w_line = 0.00045
k_sl_shell = 1.15; k_sl_inf = 1.10; k_sl_fin = 1.12; k_sl_pod = 1.15  # ★
k_sec = 0.7; k_taper = 0.6; k_ycp = 0.4; k_reinf = 0.08; k_dens = 0.95
N_bulk = 2; t_wall_pod = 0.0012   # ★
alpha_lim = math.radians(4.0)     # ★ 핀 하중 산정용 받음각 한계 [rad]

# WGHT (§8)
k_r = 0.4                 # 핀 대표 반경 계수
f_pod_prop = 0.85         # ★ 추진계 질량 중 포드에 실리는 비율 (나머지는 배선으로 동체 중앙에)
                          #   주의: k_wire=0.10 은 배선 비율 0.091 을 함의 → 0.15 와 불일치. 확정 필요

# MISS (§7)
eta_acc = 0.50            # ★ 천이 가속 실효 효율
dt_march = 1.0            # Δt [s]

# STAB (§9)
k_az = 2 * math.sqrt(2)   # ★ X 배치 전제 — 방위각 규약 승인 대기
k_q = 0.35                # ★ 천이 대표 동압 비율
d_alpha = math.radians(5.0)

# COST (§9)
k_spare = 0.2; k_misc = 0.10; k_import = 1.25   # ★
c_filament = 30000.0      # ★ [KRW/kg]
p_kWh = 130.0; N_cycle = 150; k_wear = 0.02     # 운용비 대략치
k_margin = 1.3            # ESC 정격 마진 ★
