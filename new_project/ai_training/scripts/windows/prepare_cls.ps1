$ErrorActionPreference = "Stop"

$EnvName = if ($env:CONDA_ENV) { $env:CONDA_ENV } else { "drone-yolo11" }
$CondaRoot = if ($env:CONDA_ROOT) { $env:CONDA_ROOT } else { Join-Path $env:USERPROFILE "miniconda3" }
$Python = Join-Path $CondaRoot "envs\$EnvName\python.exe"
$MaxTrain = if ($env:MAX_TRAIN_PER_CLASS) { $env:MAX_TRAIN_PER_CLASS } else { "120" }
$MaxVal = if ($env:MAX_VAL_PER_CLASS) { $env:MAX_VAL_PER_CLASS } else { "30" }
$TrainAug = if ($env:TRAIN_AUGMENTATIONS) { $env:TRAIN_AUGMENTATIONS } else { "2" }
$ValAug = if ($env:VAL_AUGMENTATIONS) { $env:VAL_AUGMENTATIONS } else { "1" }
$ImageSize = if ($env:IMAGE_SIZE) { $env:IMAGE_SIZE } else { "224" }

Set-Location (Resolve-Path "$PSScriptRoot\..\..")

if (!(Test-Path "datasets\raw\cifar-100-python")) {
  New-Item -ItemType Directory -Force -Path "datasets\raw" | Out-Null
  curl.exe -L "https://www.cs.toronto.edu/~kriz/cifar-100-python.tar.gz" -o "datasets\raw\cifar-100-python.tar.gz"
  tar -xzf "datasets\raw\cifar-100-python.tar.gz" -C "datasets\raw"
}

& $Python scripts\classification\prepare_cifar100_cls.py `
  --clean `
  --max-train-per-class $MaxTrain `
  --max-val-per-class $MaxVal `
  --train-augmentations $TrainAug `
  --val-augmentations $ValAug `
  --image-size $ImageSize
& $Python scripts\classification\check_cls_dataset.py

