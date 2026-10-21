# Ejecuta PlatformIO sin suponer que platformio.exe esta dentro del repositorio.
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('build', 'upload', 'monitor')]
    [string]$Action = 'upload',
    [string]$Port = ''
)
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath (Split-Path -Parent $PSScriptRoot)

$pioArguments = switch ($Action) {
    'build'   { @('run', '--environment', 'raton') }
    'upload'  { @('run', '--environment', 'raton', '--target', 'upload') }
    'monitor' { @('device', 'monitor', '--baud', '115200') }
}
if ($Port) {
    if ($Action -eq 'upload') { $pioArguments += @('--upload-port', $Port) }
    elseif ($Action -eq 'monitor') { $pioArguments += @('--port', $Port) }
}

function Test-PlatformIOCore {
    param([string]$Executable, [string[]]$Prefix = @())
    try {
        & $Executable @Prefix -c 'import platformio' 2>$null
        return ($LASTEXITCODE -eq 0)
    }
    catch { return $false }
}

# Primero el Python del Core que instala PlatformIO IDE. Evita lanzadores .exe
# que pueden conservar una ruta antigua cuando se mueve el entorno.
$coreDirectories = @()
if ($env:PLATFORMIO_CORE_DIR) { $coreDirectories += $env:PLATFORMIO_CORE_DIR }
if ($env:USERPROFILE) { $coreDirectories += (Join-Path $env:USERPROFILE '.platformio') }
foreach ($coreDirectory in ($coreDirectories | Select-Object -Unique)) {
    $python = Join-Path $coreDirectory 'penv\Scripts\python.exe'
    if (Test-Path -LiteralPath $python) {
        if (Test-PlatformIOCore -Executable $python) {
            Write-Host "PlatformIO Core: $python"
            & $python -m platformio @pioArguments
            exit $LASTEXITCODE
        }
    }
}

# Tambien admite una instalacion de Core independiente incluida en PATH.
foreach ($name in @('pio', 'platformio')) {
    $application = Get-Command $name -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($application) {
        & $application.Source @pioArguments
        exit $LASTEXITCODE
    }
}
$launcher = Get-Command 'py' -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
if ($launcher) {
    if (Test-PlatformIOCore -Executable $launcher.Source -Prefix @('-3')) {
        & $launcher.Source -3 -m platformio @pioArguments
        exit $LASTEXITCODE
    }
}
Write-Error ('No se encontro PlatformIO Core. Instala/habilita PlatformIO IDE en VS Code ' +
    '(extension platformio.platformio-ide), espera a que termine de instalar el Core y reinicia VS Code. ' +
    'Alternativa con Python instalado: py -3 -m pip install --user platformio. ' +
    'No copies platformio.exe al repositorio.')
exit 1
