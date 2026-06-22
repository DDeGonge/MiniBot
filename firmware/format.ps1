# Format all files recursively in the firmware directory
# Supports Python (.py) and C++ (.cpp, .h) files
# Usage: .\format.ps1

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Host "==========================================" -ForegroundColor Green
Write-Host "Formatting Python files with black..." -ForegroundColor Green
Write-Host "==========================================" -ForegroundColor Green

try {
    $blackPath = (Get-Command black -ErrorAction Stop).Source
    Get-ChildItem -Path $ScriptDir -Filter "*.py" -Recurse | ForEach-Object {
        Write-Host "Formatting: $($_.FullName)"
        & $blackPath "$($_.FullName)"
    }
}
catch {
    Write-Host "⚠️  black not found. Install with: pip install black" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "==========================================" -ForegroundColor Green
Write-Host "Formatting C++ files with clang-format..." -ForegroundColor Green
Write-Host "==========================================" -ForegroundColor Green

try {
    $clangFormatPath = (Get-Command clang-format -ErrorAction Stop).Source
    Get-ChildItem -Path $ScriptDir -Recurse | Where-Object { $_.Extension -in @('.cpp', '.h', '.hpp') } | ForEach-Object {
        Write-Host "Formatting: $($_.FullName)"
        & $clangFormatPath -i "$($_.FullName)"
    }
}
catch {
    Write-Host "⚠️  clang-format not found. Install with:" -ForegroundColor Yellow
    Write-Host "   winget install llvm" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "==========================================" -ForegroundColor Green
Write-Host "✅ Formatting complete!" -ForegroundColor Green
Write-Host "==========================================" -ForegroundColor Green
