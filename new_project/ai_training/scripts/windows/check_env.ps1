$ErrorActionPreference = "Stop"

$EnvName = if ($env:CONDA_ENV) { $env:CONDA_ENV } else { "drone-yolo11" }
$CondaRoot = if ($env:CONDA_ROOT) { $env:CONDA_ROOT } else { Join-Path $env:USERPROFILE "miniconda3" }
$EnvDir = Join-Path $CondaRoot "envs\$EnvName"
$Python = Join-Path $EnvDir "python.exe"
$Yolo = Join-Path $EnvDir "Scripts\yolo.exe"

Set-Location (Resolve-Path "$PSScriptRoot\..\..")

& $Python -c "import torch; print('torch', torch.__version__); print('cuda_available', torch.cuda.is_available()); print('cuda_version', torch.version.cuda); print('gpu', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'none')"
& $Yolo checks
