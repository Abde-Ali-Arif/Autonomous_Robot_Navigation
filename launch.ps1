# =====================================================================
# Autonomous Mobile Robot Navigation Simulation - Windows Launcher
# Launches Webots 3D Simulation and Streamlit Mission Control Station
# =====================================================================

Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host "  AUTONOMOUS MOBILE ROBOT NAVIGATION & DYNAMIC AVOIDANCE" -ForegroundColor Green
Write-Host "  B.Tech Major Engineering Project • Webots 3D Simulation" -ForegroundColor Green
Write-Host "======================================================================" -ForegroundColor Cyan

# 1. Environment and Path Verification
$ProjectRoot = $PSScriptRoot
Set-Location $ProjectRoot

$VenvPython = "$ProjectRoot\.venv\Scripts\python.exe"
if (-Not (Test-Path $VenvPython)) {
    Write-Host "[ERROR] Virtual environment python not found at: $VenvPython" -ForegroundColor Red
    Write-Host "Please create the virtual environment using Python 3.12." -ForegroundColor Yellow
    Exit 1
}

# 2. Check Webots Installation
$WebotsBin = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
if (-Not (Test-Path $WebotsBin)) {
    $WebotsBin = "C:\Program Files\Webots\webots.exe"
}
if (-Not (Test-Path $WebotsBin)) {
    Write-Host "[WARNING] Webots executable not found at standard path." -ForegroundColor Yellow
    Write-Host "Make sure Webots is installed or added to your system PATH." -ForegroundColor Yellow
} else {
    Write-Host "[SUCCESS] Located Webots: $WebotsBin" -ForegroundColor Green
}

# 3. Set Webots Environment Variables
$env:WEBOTS_HOME = "C:\Program Files\Webots"
$env:PYTHONPATH = "C:\Program Files\Webots\lib\controller\python;$ProjectRoot"
$env:PATH = "$ProjectRoot\.venv\Scripts;$ProjectRoot\.venv;C:\Program Files\Webots\msys64\mingw64\bin;C:\Program Files\Webots\lib\controller;$env:PATH"

Write-Host "[LAUNCHER] Starting Webots 3D World..." -ForegroundColor Cyan
$WorldFile = "$ProjectRoot\worlds\indoor_navigation.wbt"
if (Test-Path $WebotsBin) {
    Start-Process -FilePath $WebotsBin -ArgumentList "`"$WorldFile`""
} else {
    Start-Process webots "`"$WorldFile`""
}

Start-Sleep -Seconds 2

Write-Host "[LAUNCHER] Starting Streamlit Mission Control Dashboard..." -ForegroundColor Cyan
Start-Process -FilePath $VenvPython -ArgumentList "-m streamlit run `"$ProjectRoot\dashboard\app.py`" --server.headless=false"

Write-Host ""
Write-Host "======================================================================" -ForegroundColor Green
Write-Host "  MISSION CONTROL READY!" -ForegroundColor Yellow
Write-Host "  1. Webots 3D Simulation window will open with the indoor arena." -ForegroundColor White
Write-Host "  2. Streamlit Dashboard will open in your browser at: http://localhost:8501" -ForegroundColor White
Write-Host "  3. In Streamlit, click '▶ Start Mission' to begin autonomous navigation." -ForegroundColor White
Write-Host "  4. While moving, click '➕ Add Obstacle' to demonstrate dynamic avoidance." -ForegroundColor White
Write-Host "======================================================================" -ForegroundColor Green
