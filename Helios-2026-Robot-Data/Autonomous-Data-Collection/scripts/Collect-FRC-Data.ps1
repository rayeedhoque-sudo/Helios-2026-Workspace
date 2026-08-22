param(
    [string]$DestinationRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Stop"

function Ensure-Directory {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) {
        New-Item -ItemType Directory -Path $Path | Out-Null
    }
}

function Copy-IfPresent {
    param(
        [string]$Source,
        [string]$Destination
    )
    if (Test-Path -LiteralPath $Source) {
        Ensure-Directory -Path $Destination
        Copy-Item -LiteralPath $Source -Destination $Destination -Recurse -Force
        Write-Host "Copied: $Source -> $Destination"
    }
}

function Copy-ChildrenIfPresent {
    param(
        [string]$SourceDir,
        [string]$Pattern,
        [string]$Destination
    )
    if (Test-Path -LiteralPath $SourceDir) {
        Ensure-Directory -Path $Destination
        Get-ChildItem -LiteralPath $SourceDir -Filter $Pattern -File -ErrorAction SilentlyContinue |
            ForEach-Object {
                Copy-Item -LiteralPath $_.FullName -Destination $Destination -Force
                Write-Host "Copied: $($_.FullName) -> $Destination"
            }
    }
}

$inbox = Join-Path $DestinationRoot "inbox"
$driverStationDest = Join-Path $inbox "driver-station-logs"
$robotLogsDest = Join-Path $inbox "robot-logs"
$vendorDest = Join-Path $inbox "vendor-exports"

Ensure-Directory -Path $driverStationDest
Ensure-Directory -Path $robotLogsDest
Ensure-Directory -Path $vendorDest

$dsLogRoots = @(
    (Join-Path $env:USERPROFILE "Documents\FRC\Log Files"),
    "C:\Users\Public\Documents\FRC\Log Files"
)

foreach ($root in $dsLogRoots) {
    Copy-ChildrenIfPresent -SourceDir $root -Pattern "*.dslog" -Destination $driverStationDest
    Copy-ChildrenIfPresent -SourceDir $root -Pattern "*.dsevents" -Destination $driverStationDest
}

$robotProject = "C:\FRC\Helios-2026"
$robotData = "C:\FRC\Helios-2026-Robot-Data"

Copy-IfPresent -Source (Join-Path $robotProject "tuner-project.json") -Destination $vendorDest
Copy-IfPresent -Source (Join-Path $robotProject "src\main\deploy\pathplanner") -Destination $vendorDest
Copy-IfPresent -Source (Join-Path $robotProject "logs") -Destination $robotLogsDest
Copy-IfPresent -Source (Join-Path $robotData "Measurement-Worksheet.md") -Destination (Join-Path $inbox "measurements")

Write-Host "Collection complete. Review inbox folders under: $DestinationRoot"
