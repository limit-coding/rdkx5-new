$ErrorActionPreference = "Stop"

$Model = if ($env:MODEL) { $env:MODEL } else { "model_exports/yolo11n_det_synthetic_rtx5060/best.onnx" }
$CalibCount = if ($env:CALIB_COUNT) { [int]$env:CALIB_COUNT } else { 300 }
$CalibSource = if ($env:CALIB_SOURCE) {
  $env:CALIB_SOURCE -split "\s+"
} else {
  @("datasets/micro_drone_det/images/train", "datasets/micro_drone_det/images/val", "..\rdk_deploy\camera_check")
}

Set-Location (Resolve-Path "$PSScriptRoot\..\..")

if (!(Test-Path -LiteralPath $Model)) {
  throw "ONNX model not found: $Model"
}

$ConvertDir = "rdk_convert\yolo11_det"
$CalibDir = Join-Path $ConvertDir "calibration_images"
New-Item -ItemType Directory -Force -Path $ConvertDir | Out-Null
Copy-Item -LiteralPath $Model -Destination (Join-Path $ConvertDir "best.onnx") -Force
if (Test-Path -LiteralPath $CalibDir) {
  Remove-Item -LiteralPath $CalibDir -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $CalibDir | Out-Null

$Images = New-Object System.Collections.Generic.List[string]
foreach ($Dir in $CalibSource) {
  if (!(Test-Path -LiteralPath $Dir)) {
    continue
  }
  Get-ChildItem -LiteralPath $Dir -Recurse -File |
    Where-Object { $_.Extension -match '^\.(jpg|jpeg|png|bmp|webp)$' } |
    Sort-Object FullName |
    ForEach-Object { $Images.Add($_.FullName) }
}

if ($Images.Count -eq 0) {
  throw "No calibration images found. Set CALIB_SOURCE to folders with board-camera images."
}

$Copied = 0
foreach ($Image in $Images | Select-Object -First $CalibCount) {
  $Ext = [System.IO.Path]::GetExtension($Image)
  $Name = "calib_{0:D4}{1}" -f $Copied, $Ext
  Copy-Item -LiteralPath $Image -Destination (Join-Path $CalibDir $Name) -Force
  $Copied += 1
}

Write-Output "Prepared YOLO11 RDK conversion package:"
Write-Output "  model: $ConvertDir\best.onnx"
Write-Output "  calibration images: $CalibDir ($Copied files)"
if ($Copied -lt 50) {
  Write-Warning "Calibration image count is low. Use 100-300 real board-camera images for better quantization."
}
