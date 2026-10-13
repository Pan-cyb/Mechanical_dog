param(
    [string]$Compiler = 'G:\Qt\Tools\mingw1120_64\bin\g++.exe',
    [string]$Node = 'G:\node\node.exe'
)

$ErrorActionPreference = 'Stop'
$project = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path

function Build-And-Run([string]$Output, [string[]]$Sources) {
    & $Compiler '-std=c++17' '-Wall' '-Wextra' '-Werror' '-I' (Join-Path $project 'test\host\stubs') '-I' (Join-Path $project 'include') @Sources '-o' (Join-Path $project $Output)
    if ($LASTEXITCODE -ne 0) { throw "compile failed: $Output" }
    & (Join-Path $project $Output)
    if ($LASTEXITCODE -ne 0) { throw "test failed: $Output" }
}

Push-Location $project
try {
    Build-And-Run 'debug_web_page_response_tests.exe' @(
        'test/host/debug_web_page_response_tests.cpp','src/Port/FlashWebPageResponse.cpp')
    Build-And-Run 'preferences_calibration_store_tests.exe' @(
        'test/host/preferences_calibration_store_tests.cpp','src/Port/PreferencesCalibrationStore.cpp',
        'src/Ctrl/ChannelCalibration.cpp','src/Ctrl/ServoController.cpp')
    Build-And-Run 'pyapple_foot_trajectory_tests.exe' @(
        'test/host/pyapple_foot_trajectory_tests.cpp','src/Ctrl/PyAppleFootTrajectory.cpp')
    Build-And-Run 'host_tests.exe' @(
        'test/host/host_tests.cpp','src/Ctrl/AttitudeEstimator.cpp','src/Ctrl/ChannelCalibration.cpp',
        'src/Ctrl/Command.cpp','src/Ctrl/ServoController.cpp','src/Ctrl/RelativeServoMap.cpp',
        'src/Ctrl/V10Gait.cpp','src/Ctrl/PyAppleFootTrajectory.cpp','src/Ctrl/V10DriveControl.cpp','src/Ctrl/V10ControlProfile.cpp','src/Ctrl/V10BodyPose.cpp','src/Ctrl/V10StaticBalance.cpp','src/Ctrl/V10LegIk.cpp','src/Ctrl/V10ShankLinearMap.cpp',
        'src/Driver/Pca9685.cpp','src/Driver/HxyIcm42688p.cpp','src/UserApp/RobotApp.cpp')
    Build-And-Run 'v10_gait_tests.exe' @(
        'test/host/v10_gait_tests.cpp','src/Ctrl/V10Gait.cpp','src/Ctrl/PyAppleFootTrajectory.cpp','src/Ctrl/V10DriveControl.cpp','src/Ctrl/V10BodyPose.cpp','src/Ctrl/V10LegIk.cpp',
        'src/Ctrl/V10ShankLinearMap.cpp','src/Ctrl/ServoController.cpp')
    Build-And-Run 'v10_motion_tests.exe' @(
        'test/host/v10_motion_tests.cpp','src/Ctrl/AttitudeEstimator.cpp','src/Ctrl/ChannelCalibration.cpp',
        'src/Ctrl/Command.cpp','src/Ctrl/ServoController.cpp','src/Ctrl/RelativeServoMap.cpp',
        'src/Ctrl/V10Gait.cpp','src/Ctrl/PyAppleFootTrajectory.cpp','src/Ctrl/V10DriveControl.cpp','src/Ctrl/V10ControlProfile.cpp','src/Ctrl/V10BodyPose.cpp','src/Ctrl/V10StaticBalance.cpp','src/Ctrl/V10LegIk.cpp','src/Ctrl/V10ShankLinearMap.cpp',
        'src/UserApp/RobotApp.cpp')
    Build-And-Run 'v10_ik_tests.exe' @('test/host/v10_ik_tests.cpp','src/Ctrl/V10LegIk.cpp')
    Build-And-Run 'v10_shank_linear_map_tests.exe' @(
        'test/host/v10_shank_linear_map_tests.cpp','src/Ctrl/V10ShankLinearMap.cpp')
    Build-And-Run 'v10_body_balance_tests.exe' @(
        'test/host/v10_body_balance_tests.cpp','src/Ctrl/V10BodyPose.cpp','src/Ctrl/V10StaticBalance.cpp')
    Build-And-Run 'v10_drive_control_tests.exe' @(
        'test/host/v10_drive_control_tests.cpp','src/Ctrl/V10DriveControl.cpp')
    & $Node 'test/host/mock_web_ui_contract.cjs'
    if ($LASTEXITCODE -ne 0) { throw 'mock_web_ui_contract failed' }
    & $Node 'test/host/web_ui_smoke.cjs'
    if ($LASTEXITCODE -ne 0) { throw 'web_ui_smoke failed' }
    Write-Host 'All strict host and Web UI tests passed.'
} finally {
    Pop-Location
}
