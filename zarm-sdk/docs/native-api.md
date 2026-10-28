# Hitbot native API: signatures and evidence

This page records what the SDK knows about Hitbot's `small_scara_interface.dll`, how each fact was
established, and how sure it is. The bindings live in [`zarm/native.py`](../zarm/native.py)
(`FUNCTIONS`), and the tests check that this page names every bound function.

## The build

| | |
|---|---|
| File | `small_scara_interface.dll`, x64, plain C exports (no CLR) |
| Location in this repo | `Z-Arm Sliders/Z-Arm Sliders/Lib/` (same files in `ZArmUnityAdapter/ZArmUnityAdapter/Lib/`; moving to `legacy/csharp/`) |
| Exports | 184: 84 mangled `ControlBean` methods, 32 JNI entry points, 68 plain C functions |
| Companions | `share.dll` (shared memory) and `server.exe` (TCP server), both next to the DLL |

The three files share memory layouts, so they are verified as a set (`VERIFIED_BUNDLES`):

| File | SHA-256 |
|---|---|
| `small_scara_interface.dll` | `c8844f3c35ad581f8f0776f7e135aa988ba45e1b51ad7eaae16776aaa0d82abe` |
| `share.dll` | `a9fb4b1d152bf1cb7c3e0b595b39e98d36c690530fadf31dc20c0b1d707645c2` |
| `server.exe` | `92b12ece6b0349dfa44c1b434dcdeb0ca5d58718e6f4af61422b5cd30d7ae081` |

`NativeBackend` refuses any other set unless `allow_unverified_build=True` is passed (the hardware
smoke test accepts that flag only for static preflight). Community evidence shows that newer Hitbot
builds changed at least one signature (`robot_initial` takes a `float` `z_trail` there), so a silent
ABI mismatch is a real risk.

## Method

1. **Export table.** Parsed with the standard-library PE reader in `zarm/pe.py`. No code was run.
2. **Callee types.** The 84 `ControlBean` methods have MSVC-decorated names that encode their
   parameter types, for example `?movej_angle@ControlBean@@QEAAHMMMMMM@Z` is
   `int ControlBean::movej_angle(float, float, float, float, float, float)`.
3. **C wrappers.** Each of the 68 plain C exports was disassembled (`dumpbin /DISASM`, static).
   Most are three instructions: `this = table[card]`, then a tail jump into a `ControlBean` method,
   with the remaining registers and stack untouched. That fixes the C signature exactly:
   `(int card, <the method's parameters>)`. The rest (`robot_movej_*`/`robot_movel_*`,
   `robot_get_scara_param`, `robot_is_stop`, `robot_initial`) were traced instruction by instruction.
4. **Vendor self-consistency.** The DLL's own composite wrappers fix argument meanings. For example,
   `robot_movel_z` reads `get_scara_param` and calls `movej_angle(angle1, angle2, z, r, speed, 0)`.
5. **Cross-checks.** The x86 build in `Lib_Bak/new` decorates its `e_robot_*` copies as `__stdcall`
   with argument byte counts (`_e_robot_movej_angle@28` is 7 four-byte arguments), and two community
   `HitbotInterface` wrappers (cwsfa/hitbot-api, Apache-2.0; PKUAILab/Arm-camera-Hand-eye-Calibration)
   use the same names and argument order.

Confidence levels used below:

* **verified**: read directly from the code.
* **corroborated**: types verified, meaning established by the vendor's own call sites and matching
  external sources.
* **inferred**: types verified, meaning taken from the vendor header comments, the legacy C# code
  or the community wrappers only.
* **unknown**: types verified, meaning not established.

C types were verified for every bound function.

## Bound functions

`card` is `int` in every `robot_*` function and must be 0..255: the DLL indexes a 256-entry table
without bounds checks. All `robot_*` functions, `card_number_is_connect` and `close_server` need a
successful `net_port_initial` first, because they read shared memory it creates. ctypes silently
narrows arguments (an `int` of `2**32` becomes 0, a `float` of `1e300` becomes infinity), so
`NativeBackend` range-checks every argument before the call.

| Function | C signature | Meaning | Notes |
|---|---|---|---|
| `net_port_initial` | `int (void)` | verified | Loads `<dll dir>\share.dll`; if no server runs, starts `<dll dir>\server.exe` (`ShellExecuteA`) and polls up to 3 s. 1 = OK, -1 = failure after a **modal message box**. Same code as `net_port_initial_auto`. |
| `close_server` | `void (void)` | verified | Sets the server's shutdown flag in shared memory. |
| `card_number_is_connect` | `int (int card)` | verified | 1 connected, 0 not, 2 card outside 0..255 (unsigned compare). |
| `robot_is_connect` | `int (int card)` | verified | Same code as `card_number_is_connect`. Not used. |
| `robot_initial` | `int (int card, int generation, int z_travel_mm)` | corroborated | **`z_travel` is an `int`** (the wrapper converts it to float). Codes 0/2/11/12/105 traced; 105 also shows a modal box. 1 = accepted; 3 = encoder error per header. May replace generation and z_travel with values the machine reports. |
| `robot_unlock_position` | `int (int card)` | verified | 1 if connected (sets two shared-memory flags), else 0. |
| `robot_set_arm_length` | `void (int card, float l1_mm, float l2_mm)` | inferred | Header: l1 is "reserved, you must introduce 200", and l2 is 200 for arms with J4, so these aren't the physical link lengths. The SDK sends `ZArmConfig.vendor_arm_lengths_mm` (200, 200). Legacy code: never while homing. |
| `robot_get_scara_param` | `void (int card, float* x, float* y, float* z, float* angle1, float* angle2, float* r, bool* communicate_success, bool* initial_finish, bool* move_flag)` | corroborated | The C API has **no `servo_off_flag`** output (a dummy is passed internally). `initial_finish` is the flag whose absence makes moves return 3, so 1 means initialized; the old header comment says the opposite. |
| `robot_get_encoder_param` | `void (int card, float* x, float* y, float* z, float* angle1, float* angle2, float* r)` | inferred | Calls `get_scara_real_coor`; output order per community wrappers. |
| `robot_get_joint_state` | `int (int card, int joint)` | corroborated | Values 0..7 traced (see below). Some faults also show a **modal message box**. |
| `robot_is_collision` | `bool (int card)` | verified | Returns `bool` (only AL defined): bind with `c_bool`. |
| `robot_is_stop` | `int (int card)` | verified | 1 when `move_flag` is 0. |
| `robot_wait_stop` | `bool (int card)` | verified | **Unbounded**: loops `Sleep(10)` until stopped, no timeout. Never called by the SDK. |
| `robot_movej_angle` | `int (int card, float angle1_deg, float angle2_deg, float z_mm, float r_deg, float speed, float roughly)` | corroborated | Order fixed by `robot_movel_z`. Codes below. |
| `robot_movej_xyz` | `int (int card, float x_mm, float y_mm, float z_mm, float r_deg, float speed, float roughly)` | corroborated | Order fixed by `robot_movej_xy`. |
| `robot_movej_xyz_lr` | `int (int card, float x_mm, float y_mm, float z_mm, float r_deg, float speed, float roughly, int lr)` | inferred | `lr` goes into the vendor IK; 7 = unreachable in that configuration. **Which value means which hand is unknown**; assumed 1 = J2 ≥ 0, -1 = J2 ≤ 0, 0 = current hand (README, assumption 3). |
| `robot_movel_xyz` | `int (int card, float x_mm, float y_mm, float z_mm, float r_deg, float speed_mm_s)` | corroborated | Order fixed by `robot_movel_xy` and `robot_movel_xyz_by_offset`. |
| `robot_movel_xyz_by_offset` | `int (int card, float dx_mm, float dy_mm, float dz_mm, float dr_deg, float speed_mm_s)` | verified | Current pose plus offsets, then `movel_xyz`. Not used. |
| `robot_movej_xy` | `int (int card, float x_mm, float y_mm, float speed)` | verified | `movej_xyz` keeping z and r. Not used. |
| `robot_movel_xy` | `int (int card, float x_mm, float y_mm, float speed_mm_s)` | verified | `movel_xyz` keeping z and r. Not used. |
| `robot_movel_z` | `int (int card, float z_mm, float speed)` | verified | Despite the name, a `movej_angle` keeping the other joints. Not used. |
| `robot_movej_agnle12` | `int (int card, float angle1_deg, float angle2_deg, float speed)` | verified | Vendor typo for "angle12"; keeps z and r. Not used. |
| `robot_change_attitude` | `int (int card, float speed)` | corroborated | Switches hand configuration. No wrapper pre-checks; codes 0/1/2/3/4/6/11/101 traced. |
| `robot_stop_move` | `void (int card)` | verified | |
| `robot_emergency_stop` | `void (int card)` | inferred | Starts a detached thread inside the DLL and returns at once; its effect on the servos is unverified. Assumed to latch until the next successful `robot_initial` (README, assumption 2). |
| `robot_pause_move` | `int (int card)` | verified | 1 OK, 0 failure. |
| `robot_resume_move` | `int (int card)` | verified | 1 OK, 0 failure. |
| `robot_joint_home` | `int (int card, int joint)` | corroborated | **Clears the initialized flag on entry, before any check, so even a refused call de-initializes the arm.** 0 not connected, 2 joint outside 1..4, 3 refused by a status flag (traced); 1 = started (inferred). Moves the joint. Assumed not to set the moving flag (README, assumption 6). |
| `robot_set_drag_teach` | `bool (int card, bool enable)` | inferred | |
| `robot_get_drag_teach` | `int (int card)` | verified | 0/1. |
| `robot_set_cooperation_fun_state` | `bool (int card, bool enable)` | inferred | Legacy C# calls this `CooperationMode`. |
| `robot_get_cooperation_fun_state` | `bool (int card)` | inferred | |
| `robot_get_digital_io_in` | `int (int card, int io)` | corroborated | 3 not connected, -1 invalid io, else `NOT(controller bit)` as 0/1. Assumed, from legacy notes: 0 = 24 V present (README, assumption 8). |
| `robot_get_digital_io_out` | `int (int card, int io)` | corroborated | 3 not connected, -1 invalid io, else `NOT(controller bit)` as 0/1, read from the same status word as the inputs. Depending on two machine-type values the controller reports, the DLL also renumbers some outputs when reading them back (most combinations swap outputs 1 and 2); whether `set_digital_out` does the same wasn't traced. |
| `robot_set_digital_io_out` | `bool (int card, int io, bool value)` | inferred | |
| `robot_set_efg_state` | `int (int card, int efg_type, float distance)` | unknown | Electric gripper; assumed absent. Not used. |
| `robot_get_efg_state` | `int (int card, int* efg_type, float* distance)` | unknown | Electric gripper; assumed absent. Not used. |

## Exports that are not bound

| Export | Why |
|---|---|
| `net_port_initial_auto` | Same function as `net_port_initial`. |
| `net_port_initial_1` | Internal: takes the shared-memory pointer that `net_port_initial` obtains. |
| `set_card_number_for_muti_control` | Internal: writes a `ControlBean` field and returns the object pointer. |
| `e_net_port_initial`, `e_robot_*` (28) | `__stdcall` copies for other languages; on x64 they are the same code as `robot_*`. |
| `Java_com_hitbot_HitbotInterface_*` (32) | JNI entry points. |
| `?...@ControlBean@@...` (84) | C++ methods; reached through the C wrappers. |

## Result codes

### Moves (`robot_movej_*`, `robot_movel_*`, `robot_change_attitude`)

Checked in this order. The first three come from the C wrapper (not in `robot_change_attitude`);
the rest come from the `ControlBean` method.

| Code | `MoveResult` | Evidence |
|---|---|---|
| 102 | `COLLISION` | verified: `is_collision()` is true |
| 103 | `JOINT_NEEDS_REINITIALIZATION` | verified: some joint state is 0 |
| 105 | `JOINT_FAULT` | verified: some joint state is 7 or 8 |
| 101 | `INVALID_PARAMETER` | verified: NaN argument |
| 0 | `BUSY` | verified: the moving flag is set |
| *card id* | (none) | verified: `roughly` outside [0, 1] returns garbage, so the SDK validates it first |
| 2 | `INVALID_SPEED` | verified: speed <= 0 |
| 3 | `NOT_INITIALIZED` | verified: the initialized flag is clear |
| 6 | `SERVO_OFF` | verified flag; meaning per header |
| 11 | `MOBILE_TERMINAL_IN_CONTROL` | verified flag; meaning per header |
| 4 | `UNREACHABLE` | verified code; meaning per header |
| 7 | `HAND_CONFIGURATION_UNREACHABLE` | verified in `movej_xyz_lr`; meaning per header |
| 1 | `OK` | verified |

### `robot_get_joint_state`

| Value | `JointState` | When (traced) |
|---|---|---|
| 0 | `NEEDS_REINITIALIZATION` | initialized, but the joint's status word is not 30000 (the DLL's "joint OK" marker) |
| 1 | `NORMAL` | normal; always for joint 4 and for card 0 |
| 2 | `INVALID_JOINT` | joint outside 1..4 |
| 3 | `NOT_INITIALIZED` | the initialized flag is clear |
| 4 | `UNAVAILABLE` | a `ControlBean` flag is clear (meaning unknown) |
| 5 | `COLLISION` | collision |
| 6 | `SPECIAL_MODE` | a `ControlBean` mode flag is set (meaning unknown) |
| 7 | `FAULT` | communication fault or missing-axis alarm; shows a modal message box |
| 8 | `FAULT_8` | not produced by this build; the move wrappers treat it as a fault |

## Vendor behaviour the SDK has to live with

* **Modal message boxes.** `net_port_initial`, `robot_initial` and `robot_get_joint_state` (which
  every move wrapper calls) can show a message box that blocks the calling thread until someone
  dismisses it. Messages decoded from the DLL (GBK): "start server.exe time out"; "copy server.exe
  and share.dll to the directory"; "initialization failed, joint %d failed"; "communication
  disorder, multiple joints collided"; "communication error flag set"; "joint %d missing-axis alarm".
* **Directory.** The DLL records its own directory at load time with `GetModuleFileNameA` into a
  256-byte buffer, then loads `share.dll` and starts `server.exe` from there. Keep the path ASCII and
  under 256 characters.
* **`server.exe`** is started with the caller's working directory and, being a TCP server, triggers
  a Windows Firewall prompt the first time. The arm connects to it (PC at `192.168.0.100`).
* **No re-entrancy guarantees** are documented. The SDK serializes calls, except stops, which use a
  separate lock so they are never queued behind a blocked call. (The legacy C# app called the
  library from several threads.) `close_server` and `NativeBackend.close()` wait for calls in
  progress; the DLL is never unloaded.
