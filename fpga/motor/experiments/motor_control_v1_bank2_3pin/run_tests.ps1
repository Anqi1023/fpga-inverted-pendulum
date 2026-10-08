param([string]$Simulator='vsim')
$ErrorActionPreference='Stop'
Push-Location -LiteralPath $PSScriptRoot
try {
    New-Item -ItemType Directory -Path verification -Force | Out-Null
    $cases=@(
        @{Top='tb_shared_chain'; Marker='SHARED_CHAIN_EQUIVALENCE_PASS'},
        @{Top='tb_control_board'; Marker='CONTROL_BOARD_CONTROLS_PASS'},
        @{Top='tb_control_board_pwm'; Marker='CONTROL_BOARD_200MS_PASS ticks=5400000 pulses=199 highs=537300'}
    )
    foreach($testCase in $cases) {
        $logPath='verification/'+$testCase.Top+'.log'
        & $Simulator -c -l $logPath -do ('set test_top '+$testCase.Top+'; do run.do')
        if($LASTEXITCODE -ne 0) { throw ('Failed: '+$testCase.Top) }
        $testText=Get-Content -LiteralPath $logPath -Raw
        if(!$testText.Contains($testCase.Marker) -or $testText -match '\*\* (Error|Fatal):|# Errors: [1-9]') {
            throw ('Failed or missing PASS: '+$testCase.Top)
        }
    }
    Write-Output 'ALL_BANK2_3PIN_TESTS_PASS'
} finally { Pop-Location }
