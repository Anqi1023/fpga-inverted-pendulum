param([string]$ToolBin = '')
$ErrorActionPreference = 'Stop'
$compiler = if ($ToolBin) { Join-Path $ToolBin 'iverilog.exe' } else { 'iverilog' }
$runtime = if ($ToolBin) { Join-Path $ToolBin 'vvp.exe' } else { 'vvp' }
Push-Location $PSScriptRoot
try {
    $rtl = @('../rtl/sensor/quadrature_decoder.v', '../rtl/sensor/velocity_estimator.v', '../rtl/sensor/sensor_state.v')
    $allOutput = @()
    foreach ($bench in @('tb_quadrature_decoder', 'tb_velocity_estimator', 'tb_sensor_state')) {
        & $compiler -g2001 -Wall -s $bench -o "$bench.vvp" @rtl "$bench.v"
        if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $bench" }
        $output = @(& $runtime "$bench.vvp" +vcd)
        $exitCode = $LASTEXITCODE
        $output | ForEach-Object { Write-Host $_ }
        $allOutput += $output
        $allOutput | Set-Content -Encoding utf8 'icarus-tests.log'
        if ($exitCode -ne 0 -or ($output -match '^FAIL') -or -not ($output -match "^PASS $bench")) {
            throw "Simulation failed: $bench"
        }
    }
    Write-Host 'PASS all sensor tests'
} finally { Pop-Location }
