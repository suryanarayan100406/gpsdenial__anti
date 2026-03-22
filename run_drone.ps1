param(
    [ValidateSet('2d', '3d')]
    [string]$Mode = '3d',

    [ValidateSet('easy', 'hard', 'realistic')]
    [string]$WorldProfile = 'realistic',

    [ValidateSet('urban_3d', 'forest_3d', 'warehouse_3d', 'mixed_3d')]
    [string]$WorldType = 'urban_3d',

    [int]$WebotsPort = 1234,

    [string]$BagOutput = '',

    [switch]$Build
)

$ErrorActionPreference = 'Stop'

$repoWslPath = '/mnt/c/Users/samai/Desktop/drone'

if ($Build) {
    Write-Host 'Building drone_nav_2d package...' -ForegroundColor Cyan
    wsl.exe bash -lc "cd $repoWslPath; source /opt/ros/jazzy/setup.bash; colcon build --packages-select drone_nav_2d"
    if ($LASTEXITCODE -ne 0) {
        throw 'Build failed.'
    }
}

if ([string]::IsNullOrWhiteSpace($BagOutput)) {
    $timestamp = Get-Date -Format 'yyyyMMdd_HHmmss'
    if ($Mode -eq '2d') {
        $BagOutput = "bags/manual_run_$timestamp"
    }
    else {
        $BagOutput = "bags/${WorldType}_run_$timestamp"
    }
}

if ($Mode -eq '2d') {
    $launchCmd = "ros2 launch drone_nav_2d drone_nav_launch.py world_profile:=$WorldProfile bag_output:=$BagOutput webots_port:=$WebotsPort"
}
else {
    $launchCmd = "ros2 launch drone_nav_2d drone_nav_3d_launch.py world_type:=$WorldType bag_output:=$BagOutput webots_port:=$WebotsPort"
}

$fullCmd = "source /opt/ros/jazzy/setup.bash; source $repoWslPath/install/setup.bash; export WEBOTS_HOME=/home/surya/webots; pkill -x webots >/dev/null 2>&1 || true; sleep 1; cd $repoWslPath; $launchCmd"

Write-Host "Running mode: $Mode" -ForegroundColor Green
Write-Host "Bag output: $BagOutput" -ForegroundColor Green
Write-Host "Webots port: $WebotsPort" -ForegroundColor Green
if ($Mode -eq '2d') {
    Write-Host "World profile: $WorldProfile" -ForegroundColor Green
}
else {
    Write-Host "World type: $WorldType" -ForegroundColor Green
}

wsl.exe bash -lc "$fullCmd"
$exitCode = $LASTEXITCODE
if ($exitCode -ne 0) {
    Write-Host "Launch exited with code $exitCode" -ForegroundColor Red
    Write-Host "Tip: if Webots opened but stayed paused, press Play in Webots." -ForegroundColor Yellow
}
exit $exitCode
