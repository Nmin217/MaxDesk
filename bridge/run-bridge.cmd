@echo off
cd /d "%~dp0"
echo Summon Desk bridge starting from: %CD%
echo Node version:
node --version
echo.
node bridge_sim.js
echo.
echo Bridge exited. Read any error message above.
pause
