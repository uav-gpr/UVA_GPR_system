# docs/probe_field_test_drift.ps1
# Purpose: decide whether tests.test_field_test failures are environment drift
#          (machine state) instead of a code regression.
# Method : build an isolated sandbox under %TEMP% using the byte-exact HEAD
#          version of uav_gpr_qt.py (round-7 pre-work code), same venv and
#          offscreen env, then run the same module there.
# Verdict: HEAD version fails with the known drift signature
#          (processed/saved/enqueued < selected AND processing_queue
#          overflows/dropped non-zero) => environment drift, not regression.
#          HEAD version fully green => suspect real regression in the tree.
# git usage is read-only (show / rev-parse / hash-object). No git writes.
# Round-7 reference commit: 0e6a548 (2026-09-26).
# Captained evidence 2026-09-30: HEAD sandbox FAILED (failures=6) in 128.282s;
# round-7 code in the same sandbox FAILED (failures=6) in 128.375s.
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$probe = Join-Path $env:TEMP 'r7_drift_probe'
if (Test-Path $probe) { Remove-Item $probe -Recurse -Force }
New-Item -ItemType Directory -Path $probe | Out-Null
New-Item -ItemType Directory -Path (Join-Path $probe 'tests') | Out-Null
Push-Location $repo
try {
  & cmd /c "git show HEAD:uav_gpr_qt.py > ""$probe\uav_gpr_qt.py"""
  Copy-Item .\bscan_view.py, .\gpr_processing.py, .\acquisition_scheduler.py, .\disk_outbox.py, .\pipeline_metrics.py, .\calibration_service.py -Destination $probe
  Copy-Item .\tests\test_field_test.py -Destination (Join-Path $probe 'tests')
  Copy-Item .\librevna -Destination $probe -Recurse
  $head = (& git rev-parse HEAD:uav_gpr_qt.py).Trim()
  $blob = (& git hash-object (Join-Path $probe 'uav_gpr_qt.py')).Trim()
  if ($head -ne $blob) { throw "sandbox copy does not match HEAD ($head vs $blob)" }
  Write-Host "sandbox uav_gpr_qt.py matches HEAD blob $head"
} finally { Pop-Location }
$env:QT_QPA_PLATFORM = 'offscreen'
$env:QTWEBENGINE_DISABLE_SANDBOX = '1'
$py = Join-Path $repo '.venv312\Scripts\python.exe'
Push-Location $probe
try { & $py -m unittest tests.test_field_test } finally { Pop-Location }
Write-Host '---- verdict ----'
Write-Host '6 failures with drift signature = environment drift (HEAD code fails too).'
Write-Host 'All green = suspect real regression; go bisect the working tree.'
