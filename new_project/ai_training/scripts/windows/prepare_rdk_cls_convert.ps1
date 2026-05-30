$ErrorActionPreference = "Stop"

$EnvName = if ($env:CONDA_ENV) { $env:CONDA_ENV } else { "drone-yolo11" }
$CondaRoot = if ($env:CONDA_ROOT) { $env:CONDA_ROOT } else { Join-Path $env:USERPROFILE "miniconda3" }
$Python = Join-Path $CondaRoot "envs\$EnvName\python.exe"
$Model = if ($env:MODEL) { $env:MODEL } else { "runs/micro_drone/yolo11n_cifar100_cls_full_rtx5060/weights/best.onnx" }
$CalibCount = if ($env:CALIB_COUNT) { $env:CALIB_COUNT } else { "300" }

Set-Location (Resolve-Path "$PSScriptRoot\..\..")

$ConvertDir = Resolve-Path "rdk_convert\cifar100_cls"
Copy-Item -LiteralPath $Model -Destination (Join-Path $ConvertDir "best.onnx") -Force
& $Python scripts\rdk\prepare_cls_calibration.py --clean --count $CalibCount

Write-Output "Prepared RDK conversion package:"
Write-Output "  $ConvertDir"
Write-Output "Next step requires Docker/OpenExplorer hb_mapper."
