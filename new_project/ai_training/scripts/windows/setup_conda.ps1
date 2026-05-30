$ErrorActionPreference = "Stop"

$EnvName = if ($env:CONDA_ENV) { $env:CONDA_ENV } else { "drone-yolo11" }
$CondaRoot = if ($env:CONDA_ROOT) { $env:CONDA_ROOT } else { Join-Path $env:USERPROFILE "miniconda3" }
$Conda = Join-Path $CondaRoot "Scripts\conda.exe"

if (!(Test-Path $Conda)) {
  throw "Conda not found at $Conda. Install Miniconda first, then rerun this script."
}

Set-Location (Resolve-Path "$PSScriptRoot\..\..")

& $Conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main | Out-Null
& $Conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r | Out-Null
& $Conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/msys2 | Out-Null

$envs = & $Conda env list
if ($envs -notmatch "^\s*$EnvName\s") {
  & $Conda create -n $EnvName python=3.11 -y
}

$Python = Join-Path $CondaRoot "envs\$EnvName\python.exe"
& $Python -m pip install --upgrade pip
& $Python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
& $Python -m pip install -r requirements-yolo11.txt
& $Python -m pip install onnxslim onnxruntime-gpu

& "$PSScriptRoot\check_env.ps1"

