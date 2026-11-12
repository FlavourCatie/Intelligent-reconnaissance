param(
    [int]$Count = 4,
    [string]$Area = "-772,-1006,128,-306",
    [double]$SweepRadius = 300,
    [int]$Duration = 900,
    [int]$UgvDeadline = 800,
    [double]$ShotAt = 300,
    [string]$UavMode = "kinetic",
    [string]$Formation = "column",
    [switch]$NoSpawn,
    [switch]$KeepExisting,
    [switch]$NoView,
    [switch]$NoTeleport,
    [switch]$NoSpread,
    [switch]$NoDeploy,
    [switch]$HalfMap
)

# 战术层演示启动器：清场 -> 起 UE4/飞控/场景 -> 跑 full_mission --tactics
# 与 run_demo.ps1 的区别：固定半图任务区、带 --tactics/--zone/--formation，
# 这样能看到「开局展开 -> 前沿待机 -> 编队 -> 接敌分裂/合围」。
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File WorkPlace\run_tactics_demo.ps1 -NoSpawn
#   powershell -ExecutionPolicy Bypass -File WorkPlace\run_tactics_demo.ps1

$ErrorActionPreference = 'Stop'
$Workspace = 'C:\Users\llq-5\Desktop\IS'
$Repo      = Join-Path $Workspace 'Intelligent-reconnaissance'
$UeRoot    = Join-Path $Workspace 'environment_forclass\environment\WindowsNoEditor\WindowsNoEditor'
$Driver    = Join-Path $Workspace 'material\simu_driver_windows'
$Scene     = Join-Path $Workspace 'material\practice\practice1_Scene'
$Py        = 'C:\Users\llq-5\anaconda3\envs\kyxz\python.exe'
$LogDir    = Join-Path $Repo 'WorkPlace\logs'
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
$Stamp     = Get-Date -Format 'HHmmss'
$MissionLog = Join-Path $LogDir ("tactics_demo_$Stamp.out")
$MissionErr = $MissionLog + '.err'
$MissionPid = Join-Path $LogDir 'tactics_demo.pid'

function Step($m) { Write-Host ("[demo] " + $m) -ForegroundColor Cyan }
function Ok($m)   { Write-Host ("[ ok ] " + $m) -ForegroundColor Green }

$env:PYTHONIOENCODING = 'utf-8'
$env:MPLBACKEND = 'TkAgg'

# 半地图：西半边 = arena 的 x 方向一半，y 全高。
# 与 tools/demo_run.py 的 --region half --side west 完全一致。
$SpawnRect = "-760,-990,115,-320"
if ($HalfMap) {
    $Area = & $Py -c "import json; d=json.load(open(r'$Scene\game.json',encoding='utf-8-sig')); p=d['SUBJECT_ARENA']['arena']; xs=[q['x'] for q in p]; ys=[q['y'] for q in p]; x0=min(xs); x1=(min(xs)+max(xs))/2.0; print('%.0f,%.0f,%.0f,%.0f' % (x0,min(ys),x1,max(ys)))"
    $SpawnRect = $Area
    if (-not $PSBoundParameters.ContainsKey('Count')) { $Count = 7 }
    if (-not $PSBoundParameters.ContainsKey('Duration')) { $Duration = 1500 }
    if (-not $PSBoundParameters.ContainsKey('UgvDeadline')) { $UgvDeadline = 1200 }
    if (-not $PSBoundParameters.ContainsKey('ShotAt')) { $ShotAt = 600 }
    Write-Host ("[half] half-map area = " + $Area) -ForegroundColor Yellow
}

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
    Step ("spawning {0} enemies in {1} (2 at junctions)" -f $Count, $Area)
    & $Py (Join-Path $Repo 'WorkPlace\tools\spawn_targets.py') `
        --scene $Scene --count $Count --junction 2 --rect $SpawnRect `
        --avoid-margin 330 --off-road 14 --types dynamic.prop.soldier | Out-Host
    Ok 'enemies written to targets.json'
} else {
    Ok 'reuse current targets.json (NoSpawn)'
}

Step 'starting CarlaUE4 (-CE_D34 -ConfigName=L1)'
Start-Process -FilePath (Join-Path $UeRoot 'CarlaUE4.exe') `
    -ArgumentList '-CE_D34','-ConfigName=L1' -WorkingDirectory $UeRoot
$deadline = (Get-Date).AddSeconds(240); $ready = $false
while ((Get-Date) -lt $deadline) {
    Start-Sleep -Seconds 3
    if (Get-NetTCPConnection -State Listen -LocalPort 2000 -ErrorAction SilentlyContinue) { $ready = $true; break }
}
if (-not $ready) { throw 'CarlaUE4 did not open port 2000 in 240s' }
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
Start-Sleep -Seconds 12
Ok 'scene spawned (3 UGV + 3 UAV + enemies)'

Step 'starting tactical mission (deploy -> staging -> formation -> strike)'
$missionArgs = @(
    '-u','missions\full_mission.py',
    '--scene', $Scene,
    '--uav-mode', $UavMode,
    '--uav-route','sweep',
    '--uav-altitude','150',
    '--area', $Area,
    '--sweep-radius', "$SweepRadius",
    '--tactics',
    '--zone', $Area,
    '--formation', $Formation,
    '--min-targets','2',
    '--duration', "$Duration",
    '--ugv-deadline', "$UgvDeadline",
    '--dispatch-gap','5',
    '--shot-at', "$ShotAt",
    '--console-every','5'
)
if ($NoView) { $missionArgs += '--no-view' }
if ($NoTeleport) { $missionArgs += '--no-teleport' }
if ($NoSpread) { $missionArgs += '--no-spread' }
if ($NoDeploy) { $missionArgs += '--no-ugv-deploy' }

$p = Start-Process -FilePath $Py -ArgumentList $missionArgs `
    -WorkingDirectory (Join-Path $Repo 'WorkPlace') `
    -RedirectStandardOutput $MissionLog -RedirectStandardError $MissionErr -PassThru
Set-Content -Path $MissionPid -Value $p.Id -Encoding ASCII

Write-Host ''
Write-Host '================ tactical demo running ================' -ForegroundColor Yellow
Write-Host ("mission pid : " + $p.Id)
Write-Host ("mission log : " + $MissionLog)
Write-Host ("mission err : " + $MissionErr)
Write-Host ("minimap png : " + (Join-Path $Repo 'WorkPlace\figs'))
Write-Host ''
Write-Host 'watch progress:' -ForegroundColor White
Write-Host ("  Get-Content '{0}' -Tail 40 -Wait" -f $MissionLog)
Write-Host ''
Write-Host 'stop everything:' -ForegroundColor White
Write-Host '  Get-Process CarlaUE4,subject4,main-release,driver,model -EA 0 | Stop-Process -Force'
