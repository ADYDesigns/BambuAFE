# BambuAFE Token Extractor
# Launches a real Edge browser, you log in normally, then this script
# reads your cookies via CDP remote debugging — no WebDriver, no bot detection.
#
# Run with: powershell -ExecutionPolicy Bypass -File get_bambu_token.ps1

param(
    [string]$BambuAFE_IP = "BambuAFE-ESP32"
)

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  BambuAFE Token Extractor" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# ── Find Edge ─────────────────────────────────────────────────────────────────
$edgePaths = @(
    "C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    "C:\Program Files\Microsoft\Edge\Application\msedge.exe"
)
$edgePath = $edgePaths | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $edgePath) { Write-Host "ERROR: Microsoft Edge not found." -ForegroundColor Red; exit 1 }
Write-Host "Found Edge: $edgePath" -ForegroundColor Green

# ── Kill any existing Edge instances so we get a clean profile ────────────────
Write-Host "Closing any existing Edge windows..." -ForegroundColor Cyan
Get-Process msedge -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
Start-Sleep -Seconds 2

# ── Launch Edge with remote debugging enabled (no WebDriver) ──────────────────
$debugPort = 9222
$userDataDir = Join-Path $env:TEMP "BambuAFE_Edge_Profile"
Write-Host "Launching Edge with remote debugging on port $debugPort..." -ForegroundColor Cyan

Start-Process -FilePath $edgePath -ArgumentList @(
    "--remote-debugging-port=$debugPort",
    "--user-data-dir=`"$userDataDir`"",
    "--no-first-run",
    "--no-default-browser-check",
    "https://bambulab.com/en/sign-in"
)

Start-Sleep -Seconds 3

Write-Host ""
Write-Host "========================================" -ForegroundColor Yellow
Write-Host "  ACTION REQUIRED" -ForegroundColor Yellow
Write-Host "========================================" -ForegroundColor Yellow
Write-Host "  Edge has opened — this is a REAL browser" -ForegroundColor White
Write-Host "  session, not an automated one." -ForegroundColor White
Write-Host ""
Write-Host "  1. Complete any Cloudflare check if shown" -ForegroundColor White
Write-Host "  2. Log in with your Bambu Lab account" -ForegroundColor White
Write-Host "  3. Complete any 2FA if prompted" -ForegroundColor White
Write-Host "  4. Wait until you reach the Bambu main page" -ForegroundColor White
Write-Host "  5. Come back here and press ENTER" -ForegroundColor White
Write-Host "========================================" -ForegroundColor Yellow
Write-Host ""
Read-Host "Press ENTER once you are fully logged in"

# ── Connect to Edge via CDP to read cookies ───────────────────────────────────
Write-Host "Connecting to Edge via remote debugging..." -ForegroundColor Cyan

try {
    $targets = Invoke-RestMethod -Uri "http://localhost:$debugPort/json" -UseBasicParsing
} catch {
    Write-Host "ERROR: Could not connect to Edge remote debugging." -ForegroundColor Red
    Write-Host "Make sure Edge is still open and try again." -ForegroundColor Yellow
    exit 1
}

# Find a page target (not devtools or extension pages)
$pageTarget = $targets | Where-Object { $_.type -eq "page" } | Select-Object -First 1
if (-not $pageTarget) {
    Write-Host "ERROR: No Edge page found." -ForegroundColor Red
    exit 1
}

Write-Host "Connected to page: $($pageTarget.url)" -ForegroundColor Green

# Use CDP Network.getAllCookies via WebSocket
# PowerShell 5 doesn't have native WebSocket support so we use the CDP HTTP endpoint instead
$wsUrl = $pageTarget.webSocketDebuggerUrl

# Use .NET WebSocket to send CDP command
Add-Type -AssemblyName System.Net.WebSockets
Add-Type -AssemblyName System.Threading

$ws = New-Object System.Net.WebSockets.ClientWebSocket
$uri = New-Object System.Uri($wsUrl)
$cts = New-Object System.Threading.CancellationTokenSource

try {
    $ws.ConnectAsync($uri, $cts.Token).Wait()
} catch {
    Write-Host "ERROR: Could not open WebSocket to Edge." -ForegroundColor Red
    Write-Host $_.Exception.Message
    exit 1
}

# Send Network.getAllCookies command
$cmd = '{"id":1,"method":"Network.getAllCookies","params":{}}'
$bytes = [System.Text.Encoding]::UTF8.GetBytes($cmd)
$segment = New-Object System.ArraySegment[byte] -ArgumentList (,$bytes)
$ws.SendAsync($segment, [System.Net.WebSockets.WebSocketMessageType]::Text, $true, $cts.Token).Wait()

# Read response
$buffer = New-Object byte[] 65536
$result = New-Object System.ArraySegment[byte] -ArgumentList (,$buffer)
$recv = $ws.ReceiveAsync($result, $cts.Token).Result
$json = [System.Text.Encoding]::UTF8.GetString($buffer, 0, $recv.Count)
$ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure, "", $cts.Token).Wait()

$response = $json | ConvertFrom-Json
$cookies  = $response.result.cookies

$token  = ($cookies | Where-Object { $_.name -eq "token"   } | Select-Object -First 1).value
$userId = ($cookies | Where-Object { $_.name -eq "user_id" } | Select-Object -First 1).value

# If user_id cookie not found, fetch it from Bambu's profile API using the token
if ($token -and -not $userId) {
    Write-Host "Fetching user ID from Bambu API..." -ForegroundColor Yellow
    try {
        $profileResp = Invoke-RestMethod `
            -Uri "https://api.bambulab.com/v1/user-service/my/profile" `
            -Headers @{ Authorization = "Bearer $token" } `
            -UseBasicParsing
        $userId = $profileResp.uid ?? $profileResp.userId ?? $profileResp.data.uid ?? ""
        if ($userId) {
            Write-Host "User ID fetched: $userId" -ForegroundColor Green
        }
    } catch {
        Write-Host "API call failed: $($_.Exception.Message)" -ForegroundColor Yellow
    }
}

# Close Edge
Get-Process msedge -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue

Write-Host ""

if (-not $token -or -not $userId) {
    Write-Host "ERROR: Could not find token or user_id." -ForegroundColor Red
    Write-Host "Make sure you are fully logged in before pressing ENTER." -ForegroundColor Yellow
    exit 1
}

Write-Host "========================================" -ForegroundColor Green
Write-Host "  SUCCESS" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host ""
Write-Host "User ID : $userId" -ForegroundColor White
Write-Host "Token   : $($token.Substring(0, [Math]::Min(40, $token.Length)))..." -ForegroundColor White
Write-Host ""

# ── Send to BambuAFE ──────────────────────────────────────────────────────────
$send = Read-Host "Send token directly to BambuAFE at '$BambuAFE_IP'? (Y/N)"
if ($send -match "^[Yy]") {
    Write-Host ""
    $dashPass = Read-Host "Enter your BambuAFE dashboard password"
    $creds    = [Convert]::ToBase64String([Text.Encoding]::ASCII.GetBytes("admin:$dashPass"))
    $body     = "bambu_user_id=$userId&bambu_token=$([Uri]::EscapeDataString($token))"

    try {
        $resp = Invoke-RestMethod `
            -Uri "http://$BambuAFE_IP/config" `
            -Method Post `
            -Headers @{ Authorization = "Basic $creds" } `
            -ContentType "application/x-www-form-urlencoded" `
            -Body $body

        if ($resp.ok) {
            Write-Host "Token saved to BambuAFE successfully!" -ForegroundColor Green
        } else {
            Write-Host "Save failed: $($resp.error)" -ForegroundColor Red
        }
    } catch {
        Write-Host "ERROR: $($_.Exception.Message)" -ForegroundColor Red
        Write-Host "Paste the User ID and token into the config page manually." -ForegroundColor Yellow
    }
} else {
    Write-Host "Paste the User ID and token into the BambuAFE config page manually." -ForegroundColor Yellow
}

Write-Host ""
Write-Host "Done!" -ForegroundColor Green
