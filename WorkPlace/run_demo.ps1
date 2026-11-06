param(
    [int]$Count = 7,
    [string]$Area = "-1900,-1108,189,342",
    [double]$SweepRadius = 350,
    [int]$Duration = 3600,
    [int]$UgvDeadline = 2400,
    [double]$ShotAt = 600,
    [switch]$NoSpawn,
    [switch]$KeepExisting
)

# One-shot demo launcher: enemies -> UE4 -> flight driver -> scene -> mission
# Usage:
#   powershell -ExecutionPolicy Bypass -File WorkPlace\run_demo.ps1
#   powershell -ExecutionPolicy Bypass -File WorkPlace\run_demo.ps1 -Count 5 -NoSpawn

$ErrorActionPreference = 'Stop'
$Workspace = 'C:\Users\llq-5\Desktop\IS'
$Repo      = Join-Path $Workspace 'Intelligent-reconnaissance'
$UeRoot    = Join-Path $Workspace 'environment_forclass\environment\WindowsNoEditor\WindowsNoEditor'
$Driver    = Join-Path $Workspace 'material\simu_driver_windows'
$Scene     = Join-Path $Workspace 'material\practice\practice1_Scene'
$Py        = 'C:\Users\llq-5\anaconda3\envs\kyxz\python.exe'
$LogDir    = Join-Path $Workspace 'WorkPlace\logs'
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
$Stamp = Get-Date -Format 'HHmmss'
$MissionLog = Join-Path $LogDir ("demo_$Stamp.out")

function Step($m) { Write-Host ("[demo] " + $m) -ForegroundColor Cyan }
function Ok($m)   { Write-Host ("[ ok ] " + $m) -ForegroundColor Green }

$env:PYTHONIOENCODING = 'utf-8'
$env:MPLBACKEND = 'TkAgg'

if (-not $KeepExisting) {
    Step 'cleaning old processes (UE / vehicle / flight / SITL / mission)'
    Get-Process CarlaUE4,subject4,main-release,driver,model -ErrorAction SilentlyContinue | Stop-Process -Force
    Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
        Where-Object { $_.CommandLine -like '*full_mission*' } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    Start-Sleep -Seconds 5
}

Step 'clearing scene barricades (rands.json, backup kept as .bak)'
& $Py (Join-Path $Repo 'WorkPlace\tools\clear_obstacles.py') --scene $Scene | Out-Host
Ok 'barricades cleared'

if (-not $NoSpawn) {
    Step ("spawning {0} enemies inside task area {1}" -f $Count, $Area)
    & $Py (Join-Path $Repo 'WorkPlace\tools\spawn_targets.py') `
        --scene $Scene --count $Count --rect $Area `
        --avoid-margin $SweepRadius --off-road 14 --types dynamic.prop.soldier | Out-Host
    Ok 'enemies written to targets.json'
}

Step 'starting CarlaUE4 (-CE_D34 -ConfigName=L1)'
Start-Process -FilePath (Join-Path $UeRoot 'CarlaUE4.exe') `
    -ArgumentList '-CE_D34','-ConfigName=L1' -WorkingDirectory $UeRoot
$deadline = (Get-Date).AddSeconds(200); $ready = $false
while ((Get-Date) -lt $deadline) {
    Start-Sleep -Seconds 3
    if (Get-NetTCPConnection -State Listen -LocalPort 2000 -ErrorAction SilentlyContinue) { $ready = $true; break }
}
if (-not $ready) { throw 'CarlaUE4 did not open port 2000 in 200s' }
Ok 'UE4 ready (RPC 2000)'
Start-Sleep -Seconds 15

Step 'starting flight driver (main-release + driver, quadrotor)'
Start-Process -FilePath (Join-Path $Driver 'bin\main-release.exe') -WorkingDirectory (Join-Path $Driver 'bin')
Start-Sleep -Seconds 4
Start-Process -FilePath (Join-Path $Driver 'bin\driver\driver.exe') -WorkingDirectory (Join-Path $Driver 'bin\driver')
Start-Sleep -Seconds 8
Ok 'flight driver up'

Step 'starting scene (subject4.exe)'
Start-Process -FilePath (Join-Path $Scene 'subject4.exe') -WorkingDirectory $Scene
Start-Sleep -Seconds 10
Ok 'scene spawned (3 UGV + 3 UAV + enemies)'

Step 'starting mission (half-map sweep + UGV strike + minimap)'
$missionArgs = @(
    '-u','missions\full_mission.py',
    '--scene', $Scene,
    '--uav-mode','kinetic','--uav-route','sweep',
    '--area', $Area,
    '--sweep-radius', "$SweepRadius",
    '--min-targets','3',
    '--duration', "$Duration",
    '--ugv-deadline', "$UgvDeadline",
    '--dispatch-gap','5',
    '--shot-at', "$ShotAt"
)
Start-Process -FilePath $Py -ArgumentList $missionArgs -WorkingDirectory (Join-Path $Repo 'WorkPlace') `
    -RedirectStandardOutput $MissionLog -RedirectStandardError ($MissionLog + '.err') | Out-Null

Write-Host ''
Write-Host '================ demo running ================' -ForegroundColor Yellow
Write-Host ("mission log : " + $MissionLog)
Write-Host ("minimap png : " + (Join-Path $Repo 'WorkPlace\figs')) 
Write-Host ''
Write-Host 'watch progress:' -ForegroundColor White
Write-Host ("  Get-Content '{0}' -Tail 30" -f $MissionLog)
Write-Host ''
Write-Host 'stop everything:' -ForegroundColor White
Write-Host '  Get-Process CarlaUE4,subject4,main-release,driver,model -EA 0 | Stop-Process -Force'
