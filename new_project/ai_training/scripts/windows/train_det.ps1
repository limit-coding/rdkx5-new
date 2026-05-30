$ErrorActionPreference = "Stop"

$EnvName = if ($env:CONDA_ENV) { $env:CONDA_ENV } else { "drone-yolo11" }
$CondaRoot = if ($env:CONDA_ROOT) { $env:CONDA_ROOT } else { Join-Path $env:USERPROFILE "miniconda3" }
$EnvDir = Join-Path $CondaRoot "envs\$EnvName"
$Python = Join-Path $EnvDir "python.exe"
$Yolo = Join-Path $EnvDir "Scripts\yolo.exe"
$Model = if ($env:MODEL) { $env:MODEL } else { "yolo11n.pt" }
$Data = if ($env:DATA) { $env:DATA } else { "datasets/micro_drone_det/micro_drone_det.yaml" }
$ImgSz = if ($env:IMGSZ) { $env:IMGSZ } else { "640" }
$Epochs = if ($env:EPOCHS) { $env:EPOCHS } else { "120" }
$Batch = if ($env:BATCH) { $env:BATCH } else { "16" }
$Device = if ($env:DEVICE) { $env:DEVICE } else { "0" }
$Name = if ($env:NAME) { $env:NAME } else { "yolo11n_det" }

Set-Location (Resolve-Path "$PSScriptRoot\..\..")
$Project = if ($env:PROJECT) { $env:PROJECT } else { Join-Path (Get-Location) "runs\micro_drone" }

& $Python scripts\yolo11\check_dataset.py
& $Yolo detect train `
  model=$Model `
  data=$Data `
  imgsz=$ImgSz `
  epochs=$Epochs `
  batch=$Batch `
  device=$Device `
  project=$Project `
  name=$Name
