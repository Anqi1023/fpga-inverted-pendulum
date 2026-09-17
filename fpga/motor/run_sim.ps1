$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    & vsim -c -l simulation.log -do run_modelsim.do
    if ($LASTEXITCODE -ne 0) { throw 'ModelSim failed' }
    if (-not (Select-String simulation.log -Pattern '^# HENGXIN_MOTOR_OFFLINE_PASS$' -Quiet)) {
        throw 'Simulation did not reach PASS. Read simulation.log.'
    }
    Write-Host 'HENGXIN_MOTOR_OFFLINE_PASS'
} finally { Pop-Location }
