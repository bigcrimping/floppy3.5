# Build & run the RP2354 host tests (no board, no ARM toolchain, no pico-sdk).
# Usage:  ./build.ps1 [-Clang <path-to-clang.exe>]
param(
    [string]$Clang = "C:\Program Files\LLVM\bin\clang.exe"
)
$ErrorActionPreference = "Stop"
# No clang on this machine? TinyCC builds these too (it just needs the C11
# static asserts neutralised, as the ESP host tests do).
$cflags = @("-std=c11", "-O1", "-Wall", "-Wextra")
if (-not (Test-Path $Clang) -and (Test-Path "C:\TEMP\claude\tools\tcc\tcc.exe")) {
    $Clang  = "C:\TEMP\claude\tools\tcc\tcc.exe"
    $cflags = @("-Wall", "-D_Static_assert(c,m)=")
}
$here   = Split-Path -Parent $MyInvocation.MyCommand.Path        # test/host
$outdir = Join-Path $here "build"
New-Item -ItemType Directory -Force -Path $outdir | Out-Null

$src = Resolve-Path (Join-Path $here "..\..\src")                 # rp2354/src
$esp = Resolve-Path (Join-Path $here "..\..\..\esp32\main")

# --- Greaseweazle stream codec: the REAL src/gw_stream.c against (a) the ESP
# write-path emission contract and (b) the ESP's REAL read-path decoder
# (flux_decode.c, compiled in). Each .c gets its own gw_proto.h via -I order.
Write-Host "[test_gw_stream] compiling..." -ForegroundColor Cyan

$gwso = Join-Path $outdir "gw_stream.o"
& $Clang @cflags -c -o $gwso -I $src (Join-Path $src "gw_stream.c") | Out-Host
if ($LASTEXITCODE -ne 0) { throw "[gw_stream] compile failed ($LASTEXITCODE)" }

$fdo = Join-Path $outdir "flux_decode.o"
& $Clang @cflags -c -o $fdo -I $esp (Join-Path $esp "flux_decode.c") | Out-Host
if ($LASTEXITCODE -ne 0) { throw "[flux_decode] compile failed ($LASTEXITCODE)" }

$exe = Join-Path $outdir "test_gw_stream.exe"
& $Clang @cflags -I $src -I $esp `
    -o $exe "$here\test_gw_stream.c" $gwso $fdo | Out-Host
if ($LASTEXITCODE -ne 0) { throw "[test_gw_stream] compile failed ($LASTEXITCODE)" }
& $exe | Out-Host
$rc = [int]$LASTEXITCODE

# --- Flashing path: the GWRP mailbox contract (the REAL header both firmwares
# compile) driven by the stub's handler and the ESP's client sequence, plus
# ADIv5 request/parity vectors for the bit-banged SWD driver.
Write-Host "[test_rp_flash] compiling..." -ForegroundColor Cyan
$stub = Resolve-Path (Join-Path $here "..\..\tools\flash_stub")
$exe2 = Join-Path $outdir "test_rp_flash.exe"
# Only -I $stub: part 3 reaches the controller's diag.h and the ESP's
# gw_proto.h by explicit relative path, because both trees have a gw_proto.h
# and an include-path search would pick the wrong one (see the test source).
& $Clang @cflags -I $stub -o $exe2 "$here\test_rp_flash.c" | Out-Host
if ($LASTEXITCODE -ne 0) { throw "[test_rp_flash] compile failed ($LASTEXITCODE)" }
& $exe2 | Out-Host
if ($LASTEXITCODE -ne 0) { $rc = [int]$LASTEXITCODE }

Write-Host ""
if ($rc -eq 0) { Write-Host "==> ALL HOST TESTS PASS" -ForegroundColor Green }
else { Write-Host "==> SOME HOST TESTS FAILED" -ForegroundColor Red }
exit $rc
