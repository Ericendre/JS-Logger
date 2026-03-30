@echo off
setlocal

cd /d "%~dp0" || (
  echo [!] Dossier introuvable: %~dp0
  pause
  exit /b 1
)

start "" cmd /c "timeout /t 2 >nul & start "" http://127.0.0.1:8000"

python gkflasher_server.py --protocol kline --interface COM3 --desired-baudrate 0x5 --interval 0 --logger-config excluded_pids.yml

echo.
echo [*] Serveur arrete.
pause
endlocal
