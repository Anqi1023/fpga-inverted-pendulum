param([string]$Simulator = 'vsim')
$ErrorActionPreference = 'Stop'
Push-Location -LiteralPath $PSScriptRoot
try {
    $testCases = @(
        @{ Top='tb_protection'; Markers=@('PROTECTION_UNIT_PASS') },
        @{ Top='tb_motor_protected'; Markers=@('MOTOR_PROTECTED_INTEGRATION_PASS') },
        @{ Top='tb_motor'; Markers=@('HENGXIN_MOTOR_OFFLINE_PASS','HENGXIN_IBT2_ADAPTER_PASS') }
    )
    foreach ($testCase in $testCases) {
        $testLog = $testCase.Top + '.log'
        & $Simulator -c -l $testLog -do ('set test_top ' + $testCase.Top + '; do run_protection.do')
        if ($LASTEXITCODE -ne 0) { throw ('Simulation failed: ' + $testCase.Top) }
        $testText = Get-Content -LiteralPath $testLog -Raw
        foreach ($testMarker in $testCase.Markers) {
            if (-not $testText.Contains($testMarker)) { throw ('Missing PASS: ' + $testMarker) }
        }
        if ($testText -match '\*\* (Error|Fatal):|# Errors: [1-9]') { throw ('Simulation error in ' + $testLog) }
    }
    Write-Output 'ALL_PROTECTION_TESTS_PASS'
} finally {
    Pop-Location
}
