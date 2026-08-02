param([string]$DroneHost='192.168.1.115',[string]$RoverHost='')
$root=Split-Path -Parent $PSCommandPath
& 'C:\Users\User\AppData\Local\Programs\Python\Python313\python.exe' "$root\sync.py"
$env:SSH_ASKPASS='C:/Users/User/AppData/Local/Temp/SshAskPass.exe';$env:SSH_ASKPASS_REQUIRE='force';$env:DISPLAY='1'
$known='C:\Users\User\OneDrive\Документы\A2026\.sverk_known_hosts'
scp -r -o UserKnownHostsFile=$known -o StrictHostKeyChecking=accept-new "$root\common" "$root\drone" "$root\run.py" "$root\field.json" "sverk@$DroneHost`:/home/sverk/air_watch_simple/"
scp -o UserKnownHostsFile=$known -o StrictHostKeyChecking=accept-new "$root\drone\device.json" "sverk@$DroneHost`:/home/sverk/air_watch_simple/device.json"
if($RoverHost){ Write-Host "TODO: configure rover SSH user/path, then copy common, rover and field.json" }
