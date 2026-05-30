$ErrorActionPreference = "Stop"

$EnvName = if ($env:CONDA_ENV) { $env:CONDA_ENV } else { "drone-yolo11" }
$CondaRoot = if ($env:CONDA_ROOT) { $env:CONDA_ROOT } else { Join-Path $env:USERPROFILE "miniconda3" }
$EnvDir = Join-Path $CondaRoot "envs\$EnvName"
$Python = Join-Path $EnvDir "python.exe"
$Model = if ($env:MODEL) { $env:MODEL } else { "runs/micro_drone/yolo11n_cifar100_cls_rtx5060/weights/best.pt" }
$ImgSz = if ($env:IMGSZ) { $env:IMGSZ } else { "224" }
$Opset = if ($env:OPSET) { $env:OPSET } else { "12" }

Set-Location (Resolve-Path "$PSScriptRoot\..\..")

& $Python scripts\classification\run_cls.py export `
  --model $Model `
  --imgsz $ImgSz `
  --opset $Opset
