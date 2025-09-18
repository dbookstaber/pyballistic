<#
.SYNOPSIS
Runs the test suite across all engines with optional extended coverage.

.DESCRIPTION
- First engine runs root tests ("pytest tests") and if any fail the script stops.
  - By default runs root tests for each engine with `-m "not extended"` to skip extended tests.
  - If `-ext` is supplied, runs WITHOUT the -m filter (i.e., includes extended tests).
- Subsequent engines run only `-m "engine"` tests.
- For cythonized engines, also runs the subproject tests in py_ballisticcalc.exts/.
  When `-ext` is supplied, it overrides the exts default marker to include stress tests as well.

- Optional `-coverage` generates coverage analysis in:
  - .testall/covdata_noext/.coverage_*
  - .testall/covdata_ext/.coverage_* (when -ext)
  ... with combined coverage.xml in:
  - .testall/coverage_ext/coverage.xml
  - .testall/coverage_noext/coverage.xml

.EXAMPLES
# Default (skip extended; skip stress in exts)
./testall.ps1

# Include extended (and include stress for exts too)
./testall.ps1 -ext
#>

[CmdletBinding()] param(
    [switch]$ext,
    [switch]$coverage
)

$ErrorActionPreference = 'Stop'

$sw = [System.Diagnostics.Stopwatch]::StartNew()
# Resolve repository root (script is located under <repo>/scripts)
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")

# Results directory lives at repo root
$resultsDir = Join-Path $repoRoot ".testall"
if (-not (Test-Path $resultsDir)) { New-Item -ItemType Directory -Path $resultsDir | Out-Null }
Get-ChildItem -Path $resultsDir -Filter *.xml -ErrorAction SilentlyContinue | Remove-Item -Force -ErrorAction SilentlyContinue

# Coverage aggregation prep (optional)
$modeTag = if ($ext) { "ext" } else { "noext" }
$covDataDir = Join-Path $resultsDir ("covdata_" + $modeTag)
$covOutDir = Join-Path $resultsDir ("coverage_" + $modeTag)
if ($coverage) {
    if (-not (Test-Path $covDataDir)) { New-Item -ItemType Directory -Path $covDataDir | Out-Null }
    if (-not (Test-Path $covOutDir)) { New-Item -ItemType Directory -Path $covOutDir | Out-Null }
    Get-ChildItem -Path $covDataDir -Filter .coverage* -ErrorAction SilentlyContinue | Remove-Item -Force -ErrorAction SilentlyContinue
}

function Combine-Coverage {
    param(
        [Parameter(Mandatory=$true)][string]$DataDir,
        [Parameter(Mandatory=$true)][string]$OutDir
    )
    try {
        Write-Host "Combining coverage data..." -ForegroundColor DarkCyan
        Push-Location -Path $repoRoot
        try {
            if (Test-Path (Join-Path $repoRoot ".coverage")) {
                Remove-Item -Force (Join-Path $repoRoot ".coverage") -ErrorAction SilentlyContinue
            }
            & python -m coverage combine $DataDir | Out-Host
            & python -m coverage report -m | Out-Host
            & python -m coverage xml -o (Join-Path $OutDir "coverage.xml") | Out-Host
            & python -m coverage html -d (Join-Path $OutDir "html") | Out-Host
        } finally { Pop-Location }
        Write-Host ("Coverage output: {0}" -f $OutDir) -ForegroundColor DarkGreen
    } catch {
        Write-Host ("Coverage combine failed: {0}" -f $_.Exception.Message) -ForegroundColor Yellow
    }
}

function Parse-JUnit {
    param(
        [Parameter(Mandatory=$true)][string]$Engine,
        [Parameter(Mandatory=$true)][string]$Scope,
        [Parameter(Mandatory=$true)][string]$JunitPath,
        [Parameter(Mandatory=$true)][int]$ExitCode
    )
    $obj = [ordered]@{
        Engine=$Engine; Scope=$Scope; ExitCode=$ExitCode
        Total=0; Failures=0; Errors=0; Skipped=0; Passed=0
        FailedTests=@(); ParseStatus=""
    }
    if (Test-Path $JunitPath) {
        try {
            [xml]$xml = Get-Content -Path $JunitPath
            $suites = @()
            if ($xml.testsuite) { $suites += $xml.testsuite }
            if ($xml.testsuites) { $suites += $xml.testsuites.testsuite }
            foreach ($suite in $suites) {
                $obj.Total += [int]$suite.tests
                $obj.Failures += [int]$suite.failures
                $obj.Errors += [int]$suite.errors
                $obj.Skipped += [int]$suite.skipped
                foreach ($tc in $suite.testcase) {
                    if ($tc.failure -or $tc.error) {
                        $name = $tc.name
                        $cls = $tc.classname
                        $id = if ($cls) { "$cls::$name" } else { "$name" }
                        $obj.FailedTests += $id
                    }
                }
            }
            $obj.Passed = $obj.Total - $obj.Failures - $obj.Errors - $obj.Skipped
            $obj.ParseStatus = "parsed"
        } catch {
            $obj.ParseStatus = "xml-parse-error: $($_.Exception.Message)"
        }
    } else {
        $obj.ParseStatus = "missing-junit"
    }
    return [pscustomobject]$obj
}

function Invoke-PyTestRoot {
    param(
        [Parameter(Mandatory=$true)][string]$Engine,
        [switch]$IncludeExtended,
        [switch]$FilterEngineMarkers
    )
    Write-Host "=== Running root tests for engine: $Engine ===" -ForegroundColor Cyan
    $junit = Join-Path $resultsDir ("root_" + $Engine + ".xml")
    $testsPath = Join-Path $repoRoot "tests"
    $pytestArgs = @($testsPath, "--engine=$Engine", "--junitxml=$junit")
    # Build a single -m expression to avoid passing multiple -m flags
    $markerExpr = $null
    if ($FilterEngineMarkers) {
        if (-not $IncludeExtended) { $markerExpr = "(engine) and (not extended)" }
        else { $markerExpr = "engine" }
    } elseif (-not $IncludeExtended) {
        $markerExpr = "not extended"
    }
    if ($markerExpr) { $pytestArgs += @("-m", $markerExpr) }
    if ($coverage) {
        $covFile = Join-Path $covDataDir (".coverage_root_" + $Engine)
        $prevCov = $env:COVERAGE_FILE
        $env:COVERAGE_FILE = $covFile
        $pytestArgs += @("--cov=pyballistic", "--cov-branch", "--cov-report=")
    }
    & pytest @pytestArgs | Out-Host
    $exitCode = $LASTEXITCODE
    if ($coverage) { $env:COVERAGE_FILE = $prevCov }
    return Parse-JUnit -Engine $Engine -Scope "root" -JunitPath $junit -ExitCode $exitCode
}

function Invoke-PyTestExts {
    param(
        [Parameter(Mandatory=$true)][string]$Engine,
        [switch]$IncludeStress
    )
    Write-Host "=== Running exts tests for engine: $Engine ===" -ForegroundColor Yellow
    $extsPath = Join-Path $repoRoot "pyballistic.exts"
    Push-Location -Path $extsPath
    try {
        $junit = Join-Path $resultsDir ("exts_" + $Engine + ".xml")
        $pytestArgs = @("tests", "--engine=$Engine", "--junitxml=$junit")
        if ($IncludeStress) {
            # Override exts addopts (-m "not stress") to include stress as well
            $pytestArgs += @("-m", "stress or not stress")
        }
        if ($coverage) {
            $covFile = Join-Path $covDataDir (".coverage_exts_" + $Engine)
            $prevCov = $env:COVERAGE_FILE
            $env:COVERAGE_FILE = $covFile
            $pytestArgs += @("--cov=pyballistic_exts", "--cov-branch", "--cov-report=")
        }
    & pytest @pytestArgs | Out-Host
        $exitCode = $LASTEXITCODE
        if ($coverage) { $env:COVERAGE_FILE = $prevCov }
        return Parse-JUnit -Engine $Engine -Scope "exts" -JunitPath $junit -ExitCode $exitCode
    }
    finally { Pop-Location }
}

$engines = @(
    "cythonized_rk4_engine",
    "cythonized_euler_engine",
    "rk4_engine",
    "euler_engine",
    "scipy_engine",
    "verlet_engine"
)

$allResults = @()
$index = 0
foreach ($engine in $engines) {
    $isFirst = ($index -eq 0)
    $rootRes = Invoke-PyTestRoot -Engine $engine -IncludeExtended:$ext -FilterEngineMarkers:(!$isFirst)
    $allResults += $rootRes

    if ($isFirst) {
        $hasFailure = (
            ($rootRes.ParseStatus -eq 'parsed' -and ($rootRes.Failures -gt 0 -or $rootRes.Errors -gt 0)) -or
            ($rootRes.ParseStatus -ne 'parsed' -and $rootRes.ExitCode -ne 0)
        )
        if ($hasFailure) {
            Write-Host "First engine failed; stopping early." -ForegroundColor Red
            $sw.Stop()
            if ($coverage) { Combine-Coverage -DataDir $covDataDir -OutDir $covOutDir }
            # Reuse existing summary path for failures
            $validResults = @($rootRes)
            $failed = @($rootRes)
            Write-Host "FAILED TESTS SUMMARY" -ForegroundColor Red
            $engineName = if ($rootRes.Engine) { $rootRes.Engine } else { "(unknown)" }
            $scope = if ($rootRes.Scope) { $rootRes.Scope } else { "root" }
            $failures = if ($rootRes.Failures) { $rootRes.Failures } else { 0 }
            $errors = if ($rootRes.Errors) { $rootRes.Errors } else { 0 }
            $exit = $rootRes.ExitCode
            $status = if ($rootRes.ParseStatus) { $rootRes.ParseStatus } else { "" }
            Write-Host ("- Engine: {0} ({1}) | Failures: {2}, Errors: {3}, Exit: {4} {5}" -f $engineName, $scope, $failures, $errors, $exit, $status) -ForegroundColor Red
            if ($rootRes.FailedTests.Count -gt 0) {
                foreach ($t in $rootRes.FailedTests) { Write-Host ("    * {0}" -f $t) -ForegroundColor Red }
            } elseif ($exit -ne 0 -and $status -eq "missing-junit") {
                Write-Host "    * No JUnit report found; pytest likely failed before generating XML." -ForegroundColor Red
            }
            exit 1
        }
    }

    if ($engine -like "cythonized_*") {
        $allResults += (Invoke-PyTestExts -Engine $engine -IncludeStress:$ext)
    }
    $index += 1
}

$sw.Stop()

# Filter only structured result objects
$validResults = $allResults | Where-Object { $_ -isnot [string] -and $_ -and $_.Engine -and $_.Scope }

# Persist raw results for inspection
try {
    $jsonPath = Join-Path $resultsDir "results.json"
    $validResults | ConvertTo-Json -Depth 6 | Set-Content -Path $jsonPath -Encoding UTF8
} catch { }

# Final summary
$failed = $validResults | Where-Object {
    $_ -and $_.Engine -and $_.Scope -and (
        ($_.ParseStatus -eq 'parsed' -and ($_.Failures -gt 0 -or $_.Errors -gt 0)) -or
        ($_.ParseStatus -ne 'parsed' -and $_.ExitCode -ne 0)
    )
}
if (-not $failed) { $failed = @() }
# Deduplicate in case of accidental duplication
$failed = $failed | Group-Object Engine, Scope | ForEach-Object { $_.Group | Select-Object -First 1 }
if ($failed.Count -gt 0) {
    Write-Host "FAILED TESTS SUMMARY" -ForegroundColor Red
    foreach ($r in $failed) {
        $engineName = if ($r.Engine) { $r.Engine } else { "(unknown)" }
        $scope = if ($r.Scope) { $r.Scope } else { "root" }
        $failures = if ($r.Failures) { $r.Failures } else { 0 }
        $errors = if ($r.Errors) { $r.Errors } else { 0 }
        $exit = $r.ExitCode
        $status = if ($r.ParseStatus) { $r.ParseStatus } else { "" }
        Write-Host ("- Engine: {0} ({1}) | Failures: {2}, Errors: {3}, Exit: {4} {5}" -f $engineName, $scope, $failures, $errors, $exit, $status) -ForegroundColor Red
        if ($r.FailedTests.Count -gt 0) {
            foreach ($t in $r.FailedTests) {
                Write-Host ("    * {0}" -f $t) -ForegroundColor Red
            }
        } elseif ($exit -ne 0 -and $status -eq "missing-junit") {
            Write-Host "    * No JUnit report found; pytest likely failed before generating XML." -ForegroundColor Red
        }
    }
    exit 1
} else {
    if ($coverage) { Combine-Coverage -DataDir $covDataDir -OutDir $covOutDir }
    $totalEngRuns = ($validResults | Measure-Object).Count
    $totalTests = ($validResults | Measure-Object -Property Total -Sum).Sum
    $totalPassed = ($validResults | Measure-Object -Property Passed -Sum).Sum
    $totalSkipped = ($validResults | Measure-Object -Property Skipped -Sum).Sum
    $elapsed = $sw.Elapsed
    Write-Host "All test runs completed successfully." -ForegroundColor Green
    Write-Host ("Engine runs: {0}; Tests: {1} passed, {2} skipped; Runtime: {3} seconds" -f $totalEngRuns, $totalPassed, $totalSkipped, [Math]::Round($elapsed.TotalSeconds))
    exit 0
}
