"""COST — 비용. [골격 구현 — 단가는 SRL 스텁 의존]
가이드라인: 「COST 계산 가이드라인 — 비용」
"""
import constants as k
from interfaces import DesignVars, CostOut
from common import srl


def run(dv: DesignVars, parts, I_max, m_print, E_req_J) -> CostOut:
    C_mot = k.N_rot * parts["motor"].price
    C_prop = k.N_rot * parts["prop"].price
    C_esc = k.N_rot * srl.esc(I_max * k.k_margin).price
    C_batt = parts["batt"].price
    C_avio = sum(a[5] for a in srl.avio())
    C_parts = C_mot + C_prop + C_esc + C_batt + C_avio

    C_print = m_print * k.c_filament
    C_misc = k.k_misc * C_parts
    Cost_acq = (C_parts + C_print + C_misc) * (1.0 + k.k_spare) * k.k_import

    C_elec = (E_req_J / 3.6e6) * k.p_kWh
    C_cycle = C_batt / k.N_cycle
    C_wear = k.k_wear * C_prop
    Cost_op = C_elec + C_cycle + C_wear

    bd = {"motor": C_mot, "prop": C_prop, "esc": C_esc, "batt": C_batt,
          "avio": C_avio, "print": C_print, "misc": C_misc}
    return CostOut(Cost_acq=Cost_acq, Cost_op=Cost_op, breakdown=bd)
