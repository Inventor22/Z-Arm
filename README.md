# Z-Arm

Python control software, a simulator and the original C# experiments for a Hitbot Z-Arm SCARA
robot (assumed to be a Z-Arm 2442 with a 310 mm Z axis).

| Folder | What it is |
| --- | --- |
| [`zarm-sdk/`](zarm-sdk/README.md) | A typed Python SDK over Hitbot's native library, with a simulated arm. Standard library only. |
| [`zarm-sim/`](zarm-sim/README.md) | URDF and MuJoCo models of the arm, and a browser view of the simulated arm. |
| [`legacy/csharp/`](legacy/csharp/README.md) | The original C# and C++ projects and their vendor libraries, kept for reference. |

## Try the simulator

```powershell
cd zarm-sim
py -3.12 -m zarm_sim.server --open    # http://127.0.0.1:8770
```

Drag the joint rings, the Z handle or (in Tool mode) the tool gizmo to plan a pose, then press Move.
The first start downloads three.js; Hitbot's CAD meshes are optional
(`uv run zarm-sim/scripts/fetch_cad_meshes.py`). See the [zarm-sim README](zarm-sim/README.md).

## Status and safety

Nothing here has driven the real arm yet. The hardware details the SDK and the simulator rely on
are documented assumptions, listed in the [zarm-sdk README](zarm-sdk/README.md#default-assumptions-unverified-on-hardware).
The first hardware step is the SDK's opt-in smoke test, with someone at the emergency stop.

The legacy applications can move real hardware and are not a verified modern control or safety
system. The main WinForms solution is `legacy\csharp\Z-Arm Sliders\Z-Arm Sliders.sln`; the
[legacy project guide](legacy/csharp/README.md) has the project inventory, historical build
requirements and the original setup notes.
