"""DOE 런처 창 — 상자를 눈으로 고치고 배치를 돌린다.

    python -m doe.gui

**계산은 여기 없다.** 이 창은 `python -m doe.run ...` 명령을 조립해 자식
프로세스로 띄우고 그 출력을 받아 보여줄 뿐이다. 그래서 창으로 한 실행과 터미널로
한 실행이 **같은 것**이다 — 조립한 명령을 창에 그대로 찍어 두므로 복사해서 터미널에
붙이면 똑같이 돌아간다 (CI·원격에서는 그렇게 쓴다).

로직을 창에 복제하지 않는 이유는 §4.6(이중 구현 금지)과 같다. 창에서만 되는 실행이
생기면 로그의 출처가 둘로 갈라진다.
"""
from __future__ import annotations

import os
import queue
import re
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import filedialog, ttk

from doe import space

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 축 표시용 단위 — ICD §2 의 표기 그대로.
UNITS = {"d_body": "m", "lambda_body": "-", "S_fin": "m²", "x_fin_ratio": "-",
         "AR_fin": "-", "f_mount": "-", "n_design": "-", "d_prop": "m",
         "pd_prop": "-", "k_E": "-", "k_mot": "-"}

# 실측 기준 — 33 점 스모크에서 14 워커로 0.419 s/점(실효). 워커 수에 선형이라
# 가정해 환산한다. **대략치다** — 유효율이 낮은 상자는 탈락점이 싸서 더 빨라진다.
SEC_PER_POINT_14W = 0.419
REF_WORKERS = 14

PROG_RE = re.compile(r"^\s*(\d+)/(\d+)\s")


class App(ttk.Frame):
    def __init__(self, master: tk.Tk):
        super().__init__(master, padding=10)
        self.grid(sticky="nsew")
        master.columnconfigure(0, weight=1)
        master.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(3, weight=1)

        self.proc: subprocess.Popen | None = None
        self.q: queue.Queue = queue.Queue()
        self.lo_var: dict = {}
        self.hi_var: dict = {}
        self.last_log = ""

        self._build_top()
        self._build_axes()
        self._build_opts()
        self._build_output()
        self._fill_box("smoke")
        self.after(100, self._drain)

    # ── 상단: 상자 선택 ─────────────────────────────────────────────
    def _build_top(self):
        f = ttk.LabelFrame(self, text="탐색 상자", padding=8)
        f.grid(row=0, column=0, sticky="ew")
        self.box_var = tk.StringVar(value="smoke")
        for c, (val, txt) in enumerate((
                ("smoke", "smoke — 공칭점 ±4 % (배관 확인용)"),
                ("screen", "screen — 1차 스크리닝 (6 축이 비어 있다)"),
                ("main", "main — 본 DOE (상자를 불러온다)"))):
            ttk.Radiobutton(f, text=txt, variable=self.box_var, value=val,
                            command=lambda v=val: self._fill_box(v)).grid(
                row=0, column=c, sticky="w", padx=(0 if c == 0 else 16, 0))
        f.columnconfigure(2, weight=1)

        b = ttk.Frame(f)
        b.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        ttk.Button(b, text="상자 불러오기", command=self._load_box).grid(row=0, column=0)
        ttk.Button(b, text="상자 저장", command=self._save_box).grid(row=0, column=1,
                                                                    padx=(8, 0))
        ttk.Label(b, text="앞 실행의 매니페스트를 그대로 읽는다 — "
                          "확정한 상자를 손으로 다시 치지 않는다",
                  foreground="#777").grid(row=0, column=2, sticky="w", padx=(12, 0))
        ttk.Label(b, text=f"pd_prop 의 §2 하한 규칙 = {space.pd_prop_lo():.4f}",
                  foreground="#555").grid(row=0, column=3, sticky="e")
        b.columnconfigure(3, weight=1)

    # ── 축 표 ──────────────────────────────────────────────────────
    def _build_axes(self):
        f = ttk.LabelFrame(self, text="축 범위 — 비어 있으면 실행하지 않는다", padding=8)
        f.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        hdr = ("축", "단위", "하한", "상한", "")
        for c, t in enumerate(hdr):
            ttk.Label(f, text=t, font=("", 9, "bold")).grid(row=0, column=c,
                                                            padx=4, sticky="w")
        self.note = {}
        for r, a in enumerate(space.AXES, start=1):
            ttk.Label(f, text=a).grid(row=r, column=0, sticky="w", padx=4)
            ttk.Label(f, text=UNITS[a], foreground="#777").grid(row=r, column=1,
                                                                sticky="w", padx=4)
            self.lo_var[a] = tk.StringVar()
            self.hi_var[a] = tk.StringVar()
            for c, v in ((2, self.lo_var[a]), (3, self.hi_var[a])):
                e = ttk.Entry(f, textvariable=v, width=14)
                e.grid(row=r, column=c, padx=4, pady=1)
                v.trace_add("write", lambda *_: self._refresh())
            self.note[a] = ttk.Label(f, text="", foreground="#b00")
            self.note[a].grid(row=r, column=4, sticky="w", padx=8)

    # ── 실행 설정 ──────────────────────────────────────────────────
    def _build_opts(self):
        f = ttk.LabelFrame(self, text="실행 설정", padding=8)
        f.grid(row=2, column=0, sticky="ew", pady=(8, 0))

        self.n_var = tk.StringVar(value="30")
        self.seed_var = tk.StringVar(value="1")
        self.dup_var = tk.StringVar(value="0.01")
        self.wrk_var = tk.StringVar(value=str(max(1, (os.cpu_count() or 2) - 2)))
        self.out_var = tk.StringVar(value="runs/smoke")
        self.split_var = tk.BooleanVar(value=False)
        self.resume_var = tk.BooleanVar(value=False)
        self.ovw_var = tk.BooleanVar(value=False)
        self.ser_vars = {s: tk.BooleanVar(value=(s == 6)) for s in space.N_SER_LEVELS}

        def ent(col, label, var, w=8, tip=""):
            ttk.Label(f, text=label).grid(row=0, column=col, sticky="e", padx=(12, 2))
            e = ttk.Entry(f, textvariable=var, width=w)
            e.grid(row=0, column=col + 1, sticky="w")
            var.trace_add("write", lambda *_: self._refresh())
            if tip:
                ttk.Label(f, text=tip, foreground="#777").grid(row=1, column=col + 1,
                                                               sticky="w")
        ent(0, "표본 수/수준", self.n_var)
        ent(2, "시드", self.seed_var)
        ent(4, "복제 비율", self.dup_var, tip="순수성 감시")
        ent(6, "워커", self.wrk_var)

        ttk.Label(f, text="셀 수 n_ser").grid(row=0, column=8, sticky="e", padx=(12, 2))
        sf = ttk.Frame(f)
        sf.grid(row=0, column=9, sticky="w")
        for i, s in enumerate(space.N_SER_LEVELS):
            ttk.Checkbutton(sf, text=f"{s}S", variable=self.ser_vars[s],
                            command=self._refresh).grid(row=0, column=i)

        g = ttk.Frame(f)
        g.grid(row=2, column=0, columnspan=10, sticky="ew", pady=(8, 0))
        ttk.Label(g, text="출력 경로").grid(row=0, column=0, sticky="e")
        ttk.Entry(g, textvariable=self.out_var, width=30).grid(row=0, column=1,
                                                               sticky="w", padx=4)
        self.out_var.trace_add("write", lambda *_: self._refresh())
        ttk.Checkbutton(g, text="Ŝ 분해 (--split, +16 %)",
                        variable=self.split_var).grid(row=0, column=2, padx=(16, 0))
        ttk.Checkbutton(g, text="이어 돌리기 (--resume)",
                        variable=self.resume_var).grid(row=0, column=3, padx=(16, 0))
        # 「덮어쓰기」가 실제로는 이어붙이기였던 것을 바로잡았다. 이름과 하는 일이
        # 어긋나면 로그에 id 가 겹친 채로 쌓인다.
        ttk.Checkbutton(g, text="로그 새로 쓰기 (--overwrite · 기존 로그는 .old 로)",
                        variable=self.ovw_var).grid(row=0, column=4, padx=(16, 0))

        b = ttk.Frame(f)
        b.grid(row=3, column=0, columnspan=10, sticky="ew", pady=(10, 0))
        self.btn_dry = ttk.Button(b, text="상자 검사 (--dry-run)",
                                  command=lambda: self._start(dry=True))
        self.btn_run = ttk.Button(b, text="배치 실행", command=lambda: self._start())
        self.btn_stop = ttk.Button(b, text="중단", command=self._stop, state="disabled")
        self.btn_rep = ttk.Button(b, text="분석 보기", command=self._report,
                                  state="disabled")
        self.btn_csv = ttk.Button(b, text="CSV 내보내기", command=self._export,
                                  state="disabled")
        for i, w in enumerate((self.btn_dry, self.btn_run, self.btn_stop,
                               self.btn_rep, self.btn_csv)):
            w.grid(row=0, column=i, padx=(0, 8))
        self.est = ttk.Label(b, text="", foreground="#333")
        self.est.grid(row=0, column=5, sticky="e", padx=(16, 0))
        b.columnconfigure(5, weight=1)

    # ── 출력 ───────────────────────────────────────────────────────
    def _build_output(self):
        f = ttk.LabelFrame(self, text="실행", padding=8)
        f.grid(row=3, column=0, sticky="nsew", pady=(8, 0))
        f.columnconfigure(0, weight=1)
        f.rowconfigure(2, weight=1)

        self.cmd_txt = tk.Text(f, height=3, wrap="word", background="#f4f4f4")
        self.cmd_txt.grid(row=0, column=0, sticky="ew")
        self.cmd_txt.configure(state="disabled")

        self.bar = ttk.Progressbar(f, mode="determinate")
        self.bar.grid(row=1, column=0, sticky="ew", pady=(6, 6))

        self.log = tk.Text(f, height=16, wrap="none", background="#111",
                           foreground="#ddd", insertbackground="#ddd")
        self.log.grid(row=2, column=0, sticky="nsew")
        sb = ttk.Scrollbar(f, command=self.log.yview)
        sb.grid(row=2, column=1, sticky="ns")
        self.log.configure(yscrollcommand=sb.set)

    # ── 상자 채우기 ────────────────────────────────────────────────
    def _fill_box(self, name: str):
        box = space.BOXES[name]()
        for a in space.AXES:
            rng = box.get(a)
            self.lo_var[a].set("" if rng is None else f"{rng[0]:.6g}")
            self.hi_var[a].set("" if rng is None else f"{rng[1]:.6g}")
        if self.out_var.get().startswith("runs/"):
            self.out_var.set(f"runs/{name}")
        self._refresh()

    def _load_box(self):
        """앞 실행의 매니페스트(또는 저장해 둔 상자)를 읽어 칸을 채운다.

        본 DOE 의 상자는 스크리닝이 만들어 주는 것이다. 손으로 열한 축을 다시
        옮겨 적으면 그중 하나를 틀렸을 때 배치 전체가 조용히 어긋난다.
        """
        p = filedialog.askopenfilename(
            title="상자 또는 매니페스트 고르기",
            initialdir=os.path.join(_ROOT, "runs"),
            filetypes=[("상자·매니페스트 JSON", "*.json"), ("모든 파일", "*.*")])
        if not p:
            return
        try:
            box = space.load_box(p)
        except Exception as e:
            self._write(f"\n상자를 못 읽었다 — {type(e).__name__}: {e}\n")
            return
        for a in space.AXES:
            rng = box.get(a)
            self.lo_var[a].set("" if rng is None else f"{rng[0]:.10g}")
            self.hi_var[a].set("" if rng is None else f"{rng[1]:.10g}")
        self._write(f"\n상자를 읽었다: {p}\n")
        self._refresh()

    def _save_box(self):
        box, bad = self._read_box()
        if bad:
            self._write(f"\n비어 있는 축이 있어 저장하지 않았다: {', '.join(bad)}\n")
            return
        p = filedialog.asksaveasfilename(
            title="상자 저장", defaultextension=".json",
            initialdir=os.path.join(_ROOT, "runs"), initialfile="box.json",
            filetypes=[("JSON", "*.json")])
        if not p:
            return
        space.save_box(box, p)
        self._write(f"\n상자를 저장했다: {p}\n")

    # ── 입력 → 상자 ────────────────────────────────────────────────
    def _read_box(self):
        """창의 입력을 상자로. 못 읽는 칸은 None 으로 둔다 — 지어내지 않는다."""
        box, bad = {}, []
        for a in space.AXES:
            try:
                box[a] = (float(self.lo_var[a].get()), float(self.hi_var[a].get()))
            except ValueError:
                box[a] = None
                bad.append(a)
        return box, bad

    def _levels(self):
        return [s for s in space.N_SER_LEVELS if self.ser_vars[s].get()]

    def _refresh(self, *_):
        box, bad = self._read_box()
        for a in space.AXES:
            if a in bad:
                self.note[a]["text"] = "미정 (ICD §8 B-2) — 채워야 실행된다"
                self.note[a]["foreground"] = "#b00"
            else:
                lo, hi = box[a]
                self.note[a]["text"] = "하한 ≥ 상한" if lo >= hi else ""
                self.note[a]["foreground"] = "#b00"
        # 경고는 상자가 다 찼을 때만 (§2 하한 규칙 등)
        if not bad:
            try:
                for w in space.validate(box):
                    a = w.split()[0]
                    if a in self.note:
                        self.note[a]["text"] = "⚠ " + w[len(a) + 1:]
                        self.note[a]["foreground"] = "#a60"
            except ValueError:
                pass

        try:
            n = int(self.n_var.get())
            dup = float(self.dup_var.get())
            wrk = max(1, int(self.wrk_var.get()))
            pts = n * max(len(self._levels()), 1)
            if dup > 0:                       # sample.build 과 같은 셈 (올림이다)
                step = max(1, round(1 / dup))
                pts += -(-pts // step)
            sec = pts * SEC_PER_POINT_14W * REF_WORKERS / wrk
            self.est["text"] = (f"표본 {pts} 점 · 예상 {sec / 60:.1f} 분 "
                                f"(실측 환산, 대략치)")
        except (ValueError, ZeroDivisionError):
            self.est["text"] = ""
        self._show_cmd(self._build_cmd(False))

    # ── 명령 조립 ──────────────────────────────────────────────────
    def _build_cmd(self, dry: bool) -> list:
        box, bad = self._read_box()
        cmd = [sys.executable, "-m", "doe.run",
               "--box", self.box_var.get(),
               "--n", self.n_var.get(),
               "--n-ser", ",".join(str(s) for s in self._levels()) or "6",
               "--seed", self.seed_var.get(),
               "--dup-frac", self.dup_var.get(),
               "--workers", self.wrk_var.get(),
               "--out", self.out_var.get()]
        for a in space.AXES:
            if box[a] is not None:
                cmd += ["--set", f"{a}={box[a][0]:.10g}:{box[a][1]:.10g}"]
        if self.split_var.get():
            cmd.append("--split")
        if self.resume_var.get():
            cmd.append("--resume")
        if self.ovw_var.get():
            cmd.append("--overwrite")
        if dry:
            cmd.append("--dry-run")
        return cmd

    def _show_cmd(self, cmd: list):
        self.cmd_txt.configure(state="normal")
        self.cmd_txt.delete("1.0", "end")
        self.cmd_txt.insert("1.0", subprocess.list2cmdline(cmd[1:]).replace(
            f"{sys.executable} ", ""))
        self.cmd_txt.configure(state="disabled")

    # ── 실행 ───────────────────────────────────────────────────────
    def _start(self, dry: bool = False, cmd: list | None = None):
        if self.proc is not None:
            return
        _, bad = self._read_box()
        if bad and cmd is None:
            self._write(f"\n비어 있는 축이 있다: {', '.join(bad)}\n"
                        "  ICD §8 B-2 미확정 축이다. 값을 넣어야 표본을 뿌린다.\n")
            return
        cmd = cmd or self._build_cmd(dry)
        self._show_cmd(cmd)
        self.log.delete("1.0", "end")
        self.bar["value"] = 0
        for w in (self.btn_dry, self.btn_run, self.btn_rep, self.btn_csv):
            w["state"] = "disabled"
        self.btn_stop["state"] = "normal"

        env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1")
        flags = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0
        self.proc = subprocess.Popen(
            cmd, cwd=_ROOT, env=env, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding="utf-8",
            errors="replace", bufsize=1, creationflags=flags)
        threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()

    def _pump(self, proc):
        for line in proc.stdout:
            self.q.put(line)
        proc.wait()
        self.q.put(None)                       # 끝났다는 신호

    def _stop(self):
        if self.proc is not None:
            self.proc.terminate()
            self._write("\n[중단 요청] 여기까지는 로그에 남았다 — "
                        "「이어 돌리기」로 이어서 돌린다.\n")

    def _drain(self):
        """자식 출력을 창으로. 진행 줄은 진행률 막대로도 옮긴다."""
        try:
            while True:
                line = self.q.get_nowait()
                if line is None:
                    self.proc = None
                    for w in (self.btn_dry, self.btn_run):
                        w["state"] = "normal"
                    self.btn_stop["state"] = "disabled"
                    if os.path.exists(os.path.join(_ROOT,
                                                   self.out_var.get() + ".jsonl")):
                        self.btn_rep["state"] = "normal"
                        self.btn_csv["state"] = "normal"
                    continue
                m = PROG_RE.match(line)
                if m:
                    done, total = int(m.group(1)), int(m.group(2))
                    self.bar["maximum"] = total
                    self.bar["value"] = done
                self._write(line)
        except queue.Empty:
            pass
        self.after(100, self._drain)

    def _write(self, s: str):
        self.log.insert("end", s)
        self.log.see("end")

    def _report(self):
        log = self.out_var.get() + ".jsonl"
        self._start(cmd=[sys.executable, "-m", "doe.report", log])

    def _export(self):
        """로그를 CSV 사본으로. 원본은 JSONL 이고 분석·재개는 그쪽을 본다."""
        log = self.out_var.get() + ".jsonl"
        self._start(cmd=[sys.executable, "-m", "doe.export", log])


def main() -> int:
    root = tk.Tk()
    root.title("고속유동드론 — DOE 배치 런처")
    root.geometry("1180x860")
    App(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
