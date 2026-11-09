param(
    [string]$Version = '3.31.8',
    [ValidateSet('x86_64', 'arm64')][string]$Architecture = 'x86_64'
)
$ErrorActionPreference = 'Stop'
$Name = "cmake-$Version-windows-$Architecture.zip"
$Base = "https://github.com/Kitware/CMake/releases/download/v$Version"
$Destination = Join-Path $env:RUNNER_TEMP 'linkerbot-cmake'
# GitHub release downloads can return transient 5xx responses on hosted runners.
# Retry both assets before failing; checksum verification below remains mandatory.
$DownloadOptions = @{ MaximumRetryCount = 5; RetryIntervalSec = 5; TimeoutSec = 120 }
Invoke-WebRequest "$Base/$Name" -OutFile "$env:RUNNER_TEMP/$Name" @DownloadOptions
Invoke-WebRequest "$Base/cmake-$Version-SHA-256.txt" -OutFile "$env:RUNNER_TEMP/cmake-checksums.txt" @DownloadOptions
$Line = @(Get-Content "$env:RUNNER_TEMP/cmake-checksums.txt" | Where-Object { $_ -match " $([regex]::Escape($Name))$" })
if ($Line.Count -ne 1) { throw 'CMake checksum entry missing or ambiguous' }
$Expected = ($Line[0] -split '\s+')[0]
if ((Get-FileHash "$env:RUNNER_TEMP/$Name" -Algorithm SHA256).Hash -ne $Expected) { throw 'CMake SHA256 mismatch' }
Expand-Archive "$env:RUNNER_TEMP/$Name" -DestinationPath $Destination -Force
"$Destination/cmake-$Version-windows-$Architecture/bin" >> $env:GITHUB_PATH
Get-ComputerInfo | Select-Object WindowsProductName, WindowsVersion, OsBuildNumber
& "${env:ProgramFiles(x86)}/Microsoft Visual Studio/Installer/vswhere.exe" -latest -products '*' -format json
if ($LASTEXITCODE -ne 0) { throw 'Visual Studio discovery failed' }
python --version
if ($LASTEXITCODE -ne 0) { throw 'Python 3 is required on the Windows runner' }
