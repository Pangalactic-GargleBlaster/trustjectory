$ErrorActionPreference = "Stop"
Set-Location "C:\pixi_ws\"
@'
.\ros2-windows\local_setup.ps1;
Set-Location "C:\Users\User\Desktop\895\trustjectory";
code .
'@ | pixi shell
