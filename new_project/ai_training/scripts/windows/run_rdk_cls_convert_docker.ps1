$ErrorActionPreference = "Stop"

$Image = if ($env:RDK_TOOLCHAIN_IMAGE) { $env:RDK_TOOLCHAIN_IMAGE } else { "crpi-0uog49363mcubexr.cn-hangzhou.personal.cr.aliyuncs.com/skyxz/rdk_toolchain:v2.0" }

Set-Location (Resolve-Path "$PSScriptRoot\..\..")
$ConvertRoot = Resolve-Path "rdk_convert"

docker pull $Image
docker run --rm -i `
  --shm-size=8g `
  -v "${ConvertRoot}:/data" `
  $Image `
  bash -lc "cd /data/cifar100_cls && hb_mapper checker --model-type onnx --march bayes-e --model best.onnx && hb_mapper makertbin --model-type onnx --config cifar100_cls_bayese_224x224_nv12.yaml"

$Bin = Get-ChildItem -Path "rdk_convert\cifar100_cls\work" -Recurse -Filter "*.bin" | Sort-Object LastWriteTime -Descending | Select-Object -First 1
if (!$Bin) {
  throw "No .bin file was produced under rdk_convert\cifar100_cls\work"
}

Copy-Item -LiteralPath $Bin.FullName -Destination "..\ros2_ws\src\camera\resource\cifar100_cls.bin" -Force
Write-Output "Copied BPU model to ros2_ws\src\camera\resource\cifar100_cls.bin"
