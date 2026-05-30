$ErrorActionPreference = "Stop"

$EnvName = if ($env:CONDA_ENV) { $env:CONDA_ENV } else { "drone-yolo11" }
$CondaRoot = if ($env:CONDA_ROOT) { $env:CONDA_ROOT } else { Join-Path $env:USERPROFILE "miniconda3" }
$EnvDir = Join-Path $CondaRoot "envs\$EnvName"
$Python = Join-Path $EnvDir "python.exe"
$Model = if ($env:MODEL) { $env:MODEL } else { "yolo11n-cls.pt" }
$Data = if ($env:DATA) { $env:DATA } else { "datasets/cifar100_target_cls" }
$ImgSz = if ($env:IMGSZ) { $env:IMGSZ } else { "224" }
$Epochs = if ($env:EPOCHS) { $env:EPOCHS } else { "80" }
$Batch = if ($env:BATCH) { $env:BATCH } else { "64" }
$Device = if ($env:DEVICE) { $env:DEVICE } else { "0" }
$Project = if ($env:PROJECT) { $env:PROJECT } else { "runs/micro_drone" }
$Name = if ($env:NAME) { $env:NAME } else { "yolo11n_cifar100_cls_rtx5060" }

Set-Location (Resolve-Path "$PSScriptRoot\..\..")

& $Python scripts\classification\check_cls_dataset.py
& $Python scripts\classification\run_cls.py train `
  --model $Model `
  --data $Data `
  --imgsz $ImgSz `
  --epochs $Epochs `
  --batch $Batch `
  --device $Device `
  --project $Project `
  --name $Name
