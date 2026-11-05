@echo off
setlocal

if not exist venv (
    python -m venv venv
)

venv\Scripts\python.exe -m pip install --quiet -r requirements.txt
venv\Scripts\pyinstaller.exe --onefile --windowed --name TitanIMXDemo run.py

echo.
echo Build complete: dist\TitanIMXDemo.exe
endlocal
