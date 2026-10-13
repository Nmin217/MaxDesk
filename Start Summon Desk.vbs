Set fso = CreateObject("Scripting.FileSystemObject")
Set shell = CreateObject("WScript.Shell")
root = fso.GetParentFolderName(WScript.ScriptFullName)
nodeCmd = "cmd.exe /c cd /d """ & root & """ && node ""bridge\bridge_sim.js"" > ""bridge.log"" 2>&1"
shell.Run nodeCmd, 0, False
WScript.Sleep 1200
shell.Run "http://localhost:8787/simulator", 1, False
