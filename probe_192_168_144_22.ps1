# ============================================================
# 探测 192.168.144.22（HM30 网段新设备）
# 步骤：[0]本机IP [1]ping [2]HTTP投递JSON [3]TCP直塞JSON [4]UDP寄明信片
# 发送内容统一为: {"msg":"hello"}
# 用法: pwsh -File probe_192_168_144_22.ps1  （或双击 PowerShell 运行）
# 兼容 Windows PowerShell 5.1 与 PowerShell 7
# ============================================================
param(
    [string]$Target = "192.168.144.22",
    [int]$HttpTimeoutSec = 5,
    [int]$TcpTimeoutMs = 5000,
    [int]$ReplyWaitSec = 3
)

$ErrorActionPreference = "Continue"
$payload = '{"msg":"hello"}'
$payloadBytes = [System.Text.Encoding]::UTF8.GetBytes($payload)

function Write-Bar([string]$t) {
    Write-Host ""
    Write-Host ("=" * 62)
    Write-Host $t
    Write-Host ("=" * 62)
}

# ---------- [0] 本机在哪个网段（判断这台电脑是否接在 192.168.144.x 上） ----------
Write-Bar "[0] 本机 IP 地址（看是否在 192.168.144.x 网段里）"
$localIPs = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object { $_.IPAddress -notlike "127.*" -and $_.IPAddress -notlike "169.254.*" }
if ($localIPs) {
    $localIPs | Select-Object IPAddress, InterfaceAlias | Format-Table -AutoSize | Out-String | ForEach-Object { Write-Host $_ }
    $onSubnet = @($localIPs | Where-Object { $_.IPAddress -like "192.168.144.*" }).Count -gt 0
    if ($onSubnet) { Write-Host "本机确实在 192.168.144.x 网段里。" }
    else { Write-Host "注意：本机不在 192.168.144.x 网段里——若目标没接在同一台路由器下，所有测试都会不通。" }
} else {
    Write-Host "未取到本机 IPv4 地址。"
}

# ---------- [1] Ping（隔空喊 4 声） ----------
Write-Bar "[1] Ping $Target（喊 4 声看有没有回音）"
$ping = Test-Connection -ComputerName $Target -Count 4 -ErrorAction SilentlyContinue
$okCount = @($ping | Where-Object { $_.Status -eq 'Success' -or $_.StatusCode -eq 0 }).Count
if ($okCount -gt 0) {
    $lat = @($ping | ForEach-Object {
        if ($_.PSObject.Properties['Latency'] -and $_.Latency -ge 0) { $_.Latency }
        elseif ($_.PSObject.Properties['ResponseTime']) { $_.ResponseTime }
    } | Where-Object { $_ -ge 0 })
    $avgLat = if ($lat.Count -gt 0) { [math]::Round(($lat | Measure-Object -Average).Average, 1) } else { "?" }
    Write-Host ("PING 结果: 通 —— {0}/4 包有回音，平均延迟 {1} ms" -f $okCount, $avgLat)
} else {
    Write-Host "PING 结果: 不通 —— 4 包全部无回音（设备可能没开机/没接网线，或它关掉了 ping 应答）"
}
# ARP 邻居表补充判断：设备"装聋"还是"真不在"
$arpOut = (arp -a $Target) -join " "
if ($arpOut -match "([0-9a-fA-F]{2}[-]){5}[0-9a-fA-F]{2}") {
    Write-Host ("ARP 补充: 邻居表里登记过这台设备（网卡地址 {0}）——设备存在过，只是不回答 ping" -f $Matches[0])
} else {
    Write-Host "ARP 补充: 邻居表里没有这台设备的记录（倾向于：设备当前确实不在线）"
}

# ---------- [2] HTTP 方式投递（80 网页门，顺带试 8080 备用网页门） ----------
Write-Bar "[2] HTTP 方式投递 JSON（就像往网页服务器递数据）"
foreach ($port in @(80, 8080)) {
    $url = "http://${Target}:${port}/"
    try {
        $resp = Invoke-WebRequest -Uri $url -Method Post -Body $payload -ContentType "application/json" -TimeoutSec $HttpTimeoutSec -UseBasicParsing
        $code = [int]$resp.StatusCode
        $body = "$($resp.Content)" -replace "\s+", " "
        if ($body.Length -gt 200) { $body = $body.Substring(0, 200) + "..." }
        Write-Host ("端口 {0}: 有回应！HTTP 状态码 {1}（{2}）" -f $port, $code, $(if ($code -lt 400) { "门开着" } else { "门开着但说没这页" }))
        if ($body) { Write-Host ("       回应内容: {0}" -f $body) }
    } catch [System.Net.WebException] {
        $r = $_.Exception.Response
        if ($null -ne $r) {
            Write-Host ("端口 {0}: 门开着！HTTP 状态码 {1}（设备上有网页服务，只是不认这个请求）" -f $port, [int]$r.StatusCode)
        } else {
            Write-Host ("端口 {0}: 无回应 —— {1}" -f $port, $_.Exception.Message)
        }
    } catch {
        Write-Host ("端口 {0}: 无回应 —— {1}" -f $port, $_.Exception.Message)
    }
}

# ---------- [3] TCP 直塞 JSON（8765 号门，项目数据链路常用门） ----------
Write-Bar "[3] TCP 直塞 JSON 到 8765 端口（项目数据链路常用门）"
$client = [System.Net.Sockets.TcpClient]::new()
try {
    $iar = $client.BeginConnect($Target, 8765, $null, $null)
    if ($iar.AsyncWaitHandle.WaitOne($TcpTimeoutMs)) {
        $client.EndConnect($iar)  # 连不上会在这里抛错
        $stream = $client.GetStream()
        $stream.Write($payloadBytes, 0, $payloadBytes.Length)
        Write-Host ("已发出 {0} 字节: {1}" -f $payloadBytes.Length, $payload)
        $stream.ReadTimeout = $ReplyWaitSec * 1000
        $buf = New-Object byte[] 4096
        try {
            $n = $stream.Read($buf, 0, $buf.Length)
            $reply = [System.Text.Encoding]::UTF8.GetString($buf, 0, $n)
            Write-Host ("对方回话了（{0} 字节）: {1}" -f $n, $reply)
        } catch {
            Write-Host "发送成功，但等了 $ReplyWaitSec 秒对方没回话（收到了但没说话，也算一种回应）"
        }
    } else {
        Write-Host "8765 门没开 —— 连接超时（那台设备上大概率没有程序在这个门收数据）"
    }
} catch {
    Write-Host ("TCP 失败 —— " + $_.Exception.Message)
} finally {
    $client.Close()
}

# ---------- [4] UDP 寄明信片（不需要对方先开门，发了就走的广播式） ----------
Write-Bar "[4] UDP 方式投递 JSON 到 8765 端口（寄明信片式，不要求签收）"
$udp = [System.Net.Sockets.UdpClient]::new()
try {
    $udp.Connect($Target, 8765)
    [void]$udp.Send($payloadBytes, $payloadBytes.Length)
    Write-Host ("已发出 {0} 字节: {1}" -f $payloadBytes.Length, $payload)
    $udp.Client.ReceiveTimeout = $ReplyWaitSec * 1000
    $remoteEP = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
    try {
        $r = $udp.Receive([ref]$remoteEP)
        Write-Host ("对方回话了: " + [System.Text.Encoding]::UTF8.GetString($r))
    } catch {
        Write-Host ("发出成功；UDP 本来就不要求回执，等了 {0} 秒没回音属正常，不代表失败" -f $ReplyWaitSec)
    }
} catch {
    Write-Host ("UDP 失败 —— " + $_.Exception.Message)
} finally {
    $udp.Close()
}

# ---------- [5] 小结 ----------
Write-Bar "[5] 小结"
Write-Host "看上面各段的结果行：ping 通不通 + 哪个'门'有回应。"
Write-Host "全部无回应时最常见的原因：设备没上电/没接网线，或这台电脑不在 192.168.144.x 网段。"
