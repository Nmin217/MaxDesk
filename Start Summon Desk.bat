@echo off
setlocal
cd /d "%~dp0"
echo ========================================
echo       SUMMON DESK - STARTUP CHECK
echo ========================================
echo Project folder: %CD%
echo.
if not exist "bridge\bridge_sim.js" (
  echo ERROR: bridge\bridge_sim.js was not found.
  echo Extract the ENTIRE ZIP folder before launching.
  pause
  exit /b 1
)
where node >nul 2>nul
if errorlevel 1 (
  echo ERROR: Node.js was not found on PATH.
  echo Install Node.js 18 or newer, then close and reopen this launcher.
  pause
  exit /b 1
)
echo Node.js found:
node --version
echo Starting the bridge in a separate window...
start "Summon Desk Bridge" /min "%ComSpec%" /k ""%~dp0bridge\run-bridge.cmd""
echo Waiting 3 seconds for the server...
timeout /t 3 /nobreak >nul
start "" "http://localhost:8787/"
start "" "http://localhost:8787/simulator"
echo.
echo Browser tabs requested. Keep the bridge window running.
echo Phone URL: http://YOUR-COMPUTER-IP:8787/
echo If the browser shows an error, check the bridge window for details.
timeout /t 8
endlocal
