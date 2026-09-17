param([string]$ToolBin = '')
$ErrorActionPreference = 'Stop'
$compiler = if ($ToolBin) { Join-Path $ToolBin 'iverilog.exe' } else { 'iverilog' }
$runtime = if ($ToolBin) { Join-Path $ToolBin 'vvp.exe' } else { 'vvp' }
Push-Location $PSScriptRoot
try {
    $rtl = Get-ChildItem (Join-Path $PSScriptRoot '../rtl/sensor/*.v') | ForEach-Object FullName
    $allOutput = @()
    foreach ($bench in @('tb_quadrature_decoder', 'tb_velocity_estimator', 'tb_sensor_math', 'tb_ads1115_reader', 'tb_sensor_state', 'tb_sensor_uart_packet')) {
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
