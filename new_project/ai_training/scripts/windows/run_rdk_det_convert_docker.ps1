$ErrorActionPreference = "Stop"

$Image = if ($env:RDK_TOOLCHAIN_IMAGE) { $env:RDK_TOOLCHAIN_IMAGE } else { "crpi-0uog49363mcubexr.cn-hangzhou.personal.cr.aliyuncs.com/skyxz/rdk_toolchain:v2.0" }
$DestBin = if ($env:DEST_BIN) { $env:DEST_BIN } else { "..\ros2_ws\src\camera\resource\yolo11_det.bin" }

Set-Location (Resolve-Path "$PSScriptRoot\..\..")
$ConvertRoot = Resolve-Path "rdk_convert"

if (!(Test-Path -LiteralPath "rdk_convert\yolo11_det\best.onnx")) {
  throw "Missing rdk_convert\yolo11_det\best.onnx. Run scripts\windows\prepare_rdk_det_convert.ps1 first."
}
if (!(Test-Path -LiteralPath "rdk_convert\yolo11_det\calibration_images")) {
  throw "Missing rdk_convert\yolo11_det\calibration_images. Run scripts\windows\prepare_rdk_det_convert.ps1 first."
}

docker pull $Image
docker run --rm -i `
  --shm-size=8g `
  -v "${ConvertRoot}:/data" `
  $Image `
  bash -lc "cd /data/yolo11_det && hb_mapper checker --model-type onnx --march bayes-e --model best.onnx && hb_mapper makertbin --model-type onnx --config yolo11_det_bayese_640x640_nv12.yaml"

$Bin = Get-ChildItem -Path "rdk_convert\yolo11_det\work" -Recurse -Filter "*.bin" | Sort-Object LastWriteTime -Descending | Select-Object -First 1
if (!$Bin) {
  throw "No .bin file was produced under rdk_convert\yolo11_det\work"
}

New-Item -ItemType Directory -Force -Path (Split-Path -Parent $DestBin) | Out-Null
Copy-Item -LiteralPath $Bin.FullName -Destination $DestBin -Force
Write-Output "Copied BPU model to $DestBin"
