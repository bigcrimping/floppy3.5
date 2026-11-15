# Build & run the host tests (no board, no ESP-IDF build system).
# Usage:  ./build.ps1 [-Tcc <path-to-tcc.exe-or-gcc>] [-Idf <esp-idf path>]
param(
    [string]$Tcc = "C:\TEMP\claude\tools\tcc\tcc.exe",
    [string]$Idf = $(if ($env:IDF_PATH) { $env:IDF_PATH } else { "C:\Espressif\frameworks\esp-idf-v5.2.7" })
)
$ErrorActionPreference = "Stop"
$here   = Split-Path -Parent $MyInvocation.MyCommand.Path        # test/host
$main   = Resolve-Path (Join-Path $here "..\..\main")
$shims  = Join-Path $here "shims"
$ff     = Join-Path $Idf "components\fatfs"
$outdir = Join-Path $here "build"
New-Item -ItemType Directory -Force -Path $outdir | Out-Null

# Host build only: neutralise C11 static asserts (kept under the real IDF build).
$common = @("-D_Static_assert(c,m)=")

function Build-Run($name, $incs, $srcs) {
    $exe = Join-Path $outdir "$name.exe"
    $args = $common + $incs + @("-o", $exe) + $srcs
    Write-Host "[$name] compiling..." -ForegroundColor Cyan
    & $Tcc @args | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "[$name] compile failed ($LASTEXITCODE)" }
    & $exe | Out-Host
    return [int]$LASTEXITCODE
}

$rc = 0

# --- Test 1: flux + MFM + CRC decode pipeline ---
# Compiles the PRODUCTION encoder (main\mfm_encode.c) so the round-trip tests
# exercise the code that actually writes disks. (A stale test-local copy with
# divergent gaps/API was deleted 2026-07-28.)
$rc += Build-Run "test_decode" `
    @("-I$shims", "-I$main", "-I$here") `
    @("$main\crc16.c", "$main\flux_decode.c", "$main\mfm_decode.c",
      "$here\disk_image_stub.c", "$main\mfm_encode.c", "$here\test_main.c")

Write-Host ""

# --- Test 2: FAT12 read path (real ff.c + fat12_diskio.c) ---
# `#include "ffconf.h"` inside ff.h resolves to ff.h's own directory first, so
# stage ff.c/ff.h/diskio.h next to our host ffconf.h to shadow the IDF one.
# Needs an ESP-IDF checkout for FatFs sources; skipped (with a warning) when
# absent so the IDF-independent tests still run on any machine.
if (Test-Path "$ff\src\ff.c") {
    $stage = Join-Path $outdir "ffstage"
    New-Item -ItemType Directory -Force -Path $stage | Out-Null
    Copy-Item "$ff\src\ff.c", "$ff\src\ff.h", "$ff\src\diskio.h" $stage -Force
    Copy-Item "$here\ffconf.h" $stage -Force
    # -U_WIN32 makes ff.h take its C99 type branch (DWORD=uint32_t) instead of the
    # windows.h branch. The IDF diskio_impl.h is shadowed by our shim in $shims
    # (tcc/C99 rejects its duplicate DWORD typedef), so $ff\diskio is NOT on the path.
    $rc += Build-Run "test_fat12" `
        @("-U_WIN32", "-I$stage", "-I$shims", "-I$main", "-I$here") `
        @("$stage\ff.c", "$here\ff_diskio_host.c", "$main\fat12_diskio.c",
          "$here\disk_image_stub.c", "$here\fat12_image.c", "$here\test_fat12.c")

    Write-Host ""

    # --- Test 2b: SD content index (real sd_index.c; the card is a scratch
    # folder under build\, SHA-256 and the POSIX gaps come from the shims) ---
    $rc += Build-Run "test_sd_index" `
        @("-U_WIN32", "-include", "$shims\host_compat.h",
          "-I$stage", "-I$shims", "-I$main", "-I$here") `
        @("$stage\ff.c", "$here\ff_diskio_host.c", "$main\fat12_diskio.c",
          "$here\disk_image_stub.c", "$here\fat12_image.c", "$main\sd_index.c",
          "$here\test_sd_index.c")
} else {
    Write-Host "[test_fat12] SKIPPED - ESP-IDF FatFs not found at $ff (set -Idf or IDF_PATH)" -ForegroundColor Yellow
}

Write-Host ""

# --- Test 3: web.c pure helpers (MIME + filename safety) ---
$rc += Build-Run "test_web_util" `
    @("-I$main", "-I$here") `
    @("$main\web_util.c", "$here\test_web_util.c")

Write-Host ""

# --- Test 3b: the order of ops waiting for the gw task ---
$rc += Build-Run "test_op_queue" `
    @("-I$main", "-I$here") `
    @("$main\op_queue.c", "$here\test_op_queue.c")

Write-Host ""

# --- Test 4: FAT12 used-track scan (--fat-aware selection) ---
$rc += Build-Run "test_fat_scan" `
    @("-I$shims", "-I$main", "-I$here") `
    @("$main\fat_scan.c", "$here\disk_image_stub.c", "$here\fat12_image.c",
      "$here\test_fat_scan.c")

Write-Host ""
if ($rc -eq 0) { Write-Host "==> ALL HOST TESTS PASS" -ForegroundColor Green }
else { Write-Host "==> SOME HOST TESTS FAILED" -ForegroundColor Red }
exit $rc
