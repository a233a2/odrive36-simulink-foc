param(
    [ValidateSet('USB','SWD')][string]$Interface = 'USB',
    [switch]$List,
    [string]$Image
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$programmer = Join-Path $env:USERPROFILE '.eide\tools\st_cube_programer\bin\STM32_Programmer_CLI.exe'
if (!(Test-Path -LiteralPath $programmer)) { throw "STM32CubeProgrammer not found: $programmer" }
if ($List) { & $programmer --list; exit $LASTEXITCODE }
if (!$Image) { $Image = Join-Path $projectRoot 'build\Debug\CM_Odrive.hex' }
$resolvedImage = (Resolve-Path -LiteralPath $Image).Path
if ($Interface -eq 'USB') {
    # ROM DFU programming only. Restore RUN and power-cycle afterwards.
    & $programmer -c port=USB1 -d $resolvedImage -v
} else {
    & $programmer -c port=SWD mode=UR -d $resolvedImage -v -rst
}
if ($LASTEXITCODE -ne 0) { throw "Flash/verification failed (exit $LASTEXITCODE)" }
Write-Host 'Flash verified. For USB DFU, restore RUN and power-cycle. Firmware starts with power stage OFF.'
