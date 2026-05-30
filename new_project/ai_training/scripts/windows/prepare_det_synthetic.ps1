$ErrorActionPreference = "Stop"

$EnvName = if ($env:CONDA_ENV) { $env:CONDA_ENV } else { "drone-yolo11" }
$CondaRoot = if ($env:CONDA_ROOT) { $env:CONDA_ROOT } else { Join-Path $env:USERPROFILE "miniconda3" }
$Python = Join-Path $CondaRoot "envs\$EnvName\python.exe"
$TrainCount = if ($env:TRAIN_COUNT) { $env:TRAIN_COUNT } else { "1400" }
$ValCount = if ($env:VAL_COUNT) { $env:VAL_COUNT } else { "350" }
$ImageSize = if ($env:IMAGE_SIZE) { $env:IMAGE_SIZE } else { "640" }

Set-Location (Resolve-Path "$PSScriptRoot\..\..")

& $Python scripts\yolo11\prepare_synthetic_det.py `
  --clean `
  --train-count $TrainCount `
  --val-count $ValCount `
  --image-size $ImageSize
& $Python scripts\yolo11\check_dataset.py

