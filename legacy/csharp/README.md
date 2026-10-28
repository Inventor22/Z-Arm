# Legacy C# Z-Arm projects

These are the original Windows control applications and experiments, retained for
reference. They have not been modernized. Their source, bundled vendor binaries,
and relative directory layout are preserved, including the C++ projects used by
the SCARA interop experiments.

| Folder | Contents |
| --- | --- |
| `Z-Arm Sliders` | WinForms control panel and bundled Hitbot libraries |
| `Z-Arm Class` | .NET Core 2.1 control wrapper and console experiment |
| `ZArm Class Framework` | .NET Framework 4.6.1 control wrapper and console experiment |
| `ZArmUnityAdapter` | .NET Framework 4.6 Unity adapter |
| `TestZArmAdapter` | Adapter test application |
| `ZArm_Test` | Earlier .NET Framework 4.6.1 WinForms experiment |
| `ScaraTesting` | .NET Core 2.0 interop experiment and native C++ projects |

Open each solution from its own folder. For the sliders application, the path
from the repository root is `legacy\csharp\Z-Arm Sliders\Z-Arm Sliders.sln`.
Keep these folders together: `TestZArmAdapter` references build output from its
sibling `ZArmUnityAdapter` folder.

## Historical build requirements

The projects target old .NET Framework/.NET Core versions; the native projects
reference the Visual Studio 2017 v141 toolset and Windows SDK 10.0.15063.0.
The sliders project restores Newtonsoft.Json from NuGet: `packages.config` pins 13.0.1 (a Dependabot
security bump), but the project's reference still points at the 12.0.2 package folder.
The Unity projects reference assemblies under `C:\Program Files\Unity`, and
`TestZArmAdapter` references the adapter's `obj\x64\Debug` output. `ZArm_Test`
expects `ClassLibrary_ControlBean.dll` in its `bin\x86\Debug` directory.
These are pre-existing, machine-specific requirements, not a portable build setup.
Vendor DLLs are stored with Git LFS; a fresh clone needs Git LFS and
`git lfs pull` to obtain the binaries rather than pointer files.

## Historical hardware setup

The instructions below are retained as historical notes, not a verified current
commissioning procedure. Confirm the network settings, SDK compatibility, and
calibration procedure against the documentation for your exact controller before
use. In particular, the old notes label `255.255.255.0` as a gateway; it is a
subnet mask. These applications can unlock, home, or move the arm; do not run them
merely to inspect the archive. Use the physical E-stop and appropriate workspace
safeguards when commissioning hardware.

### Original getting-started notes

1. Download instructional videos from: https://drive.google.com/file/d/1sNxaI04Kmc81_ty_rf0sX4ocHy2zPodI/view
2. Download and extract "_Hitbot2.9.2-CCD.4 - release.rar_" from https://drive.google.com/drive/folders/1JE9ChiKHla_VIUCflrZ7iTQbX0Gr3jJS
3. Set your IP Address to 192.168.0.100, Gateway to 255.255.255.0, and DNS to 192.168.0.1, according to video _3.1 Connection with PC_.
4. Plug in power cable to Z-Arm.
5. Plug in ethernet cable from PC to Z-Arm.
6. Open `legacy\csharp\Z-Arm Sliders\Z-Arm Sliders.sln` in Visual Studio, run the program.
7. Hit 'Connect'. You may need to retry.  Allow public and private access if you receive a firewall prompt.
8. Wait for the program to initialize and connect to the Z-Arm.
8. Try moving one of the sliders, if the arm moves, great! If not, follow step 9.
9. Make note of your Z-Arm id in the _Z-Arm Sliders_ program. Close the program. Navigate to _Hitbot2.9.2-CCD.4 - release_ and run _Joint_home.exe_. Calibrate each joint one by one. Retry step 6. If the arm does not move, unplug power and retry.

### Original troubleshooting note
I'll upload a PDF a little later.
