@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
title 고속유동드론 - DOE 런처

rem ── 더블클릭하면 DOE 런처 창이 뜬다. 계산은 창이 아니라 doe.run 이 한다.
rem    터미널에서는 이것과 똑같이:  python -m doe.gui

rem ── 콘솔용 파이썬 (사전 점검에 쓴다) ──
set "PY="
for %%C in (python.exe py.exe) do (
  if not defined PY ( where %%C >nul 2>&1 && set "PY=%%C" )
)
if not defined PY goto :nopython

rem ── 창용 파이썬 — 콘솔 창 없이 GUI 만 띄운다 ──
set "PYW="
for %%C in (pythonw.exe pyw.exe) do (
  if not defined PYW ( where %%C >nul 2>&1 && set "PYW=%%C" )
)
if not defined PYW set "PYW=%PY%"

rem ── 사전 점검 — 여기서 걸리면 창이 조용히 안 뜨는 대신 이유가 보인다 ──
"%PY%" -c "import tkinter, doe.gui" 2>"%TEMP%\doe_gui_check.txt"
if errorlevel 1 goto :importfail

start "" "%PYW%" -m doe.gui
exit /b 0

:nopython
echo.
echo   파이썬을 찾지 못했다.
echo   Python 3.11 이상을 설치하고 PATH 에 넣는다 (설치 화면의
echo   "Add python.exe to PATH" 를 켜면 된다).
echo.
pause
exit /b 1

:importfail
echo.
echo   런처를 띄우지 못했다. 이유는 아래와 같다.
echo.
type "%TEMP%\doe_gui_check.txt"
echo.
echo   tkinter 가 없다면 파이썬을 설치할 때 tcl/tk 를 뺀 것이다 - 다시 설치한다.
echo   그 밖의 오류는 이 창째로 캡처해서 보고한다.
echo.
pause
exit /b 1
