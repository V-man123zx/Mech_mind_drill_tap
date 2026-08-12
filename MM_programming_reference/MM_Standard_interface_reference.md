# Mech-Mind Standard Interface — Engineering Reference

**Doc baseline:** Mech-Mind Software Suite / Robot Integration docs **v2.2.1 ("latest")**, plus **v1.7.4** for legacy naming. Every page below was fetched directly and read in full.

> **Critical version warning:** Command *numbers* are largely stable but **official command names and the command set itself changed substantially between 1.7.x and 2.x**. If your handbook targets a 1.7.x site, use the 1.7.4 names column. See §1.3.

---

## 1. Command set — TCP/IP Standard Interface

Source: [TCP/IP Interface Commands (2.2.1)](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-development-manual/tcp-ip-socket.html)

### 1.1 Conventions (verbatim from docs)

- **Direction:** the robot / PLC / host computer is the **client**; Mech-Mind Vision System is the **server**. The client always initiates; the vision system only replies. It never pushes unsolicited data (exception: Command 601, see §1.4).
- **Formats:** ASCII or HEX. ASCII delimiter = English comma, terminator = `\r` (e.g. `103,1,2\r`). HEX is **fixed 64 bytes**, big-endian or little-endian; short data is zero-padded, extra fields ignored.
- **Units:** joint positions in degrees. Flange/tool pose = `[x, y, z, a, b, c]`, x/y/z in **mm**, a/b/c as **Euler angles in degrees**. *"The format of the Euler angles for the input flange pose corresponds to the selected robot brand."*
- **Vision point** = an object recognized by Mech-Vision (pose, label, dimensions, custom data). **Waypoint** = a point the robot reaches along the planned path (pose, label, motion type); split into *Vision Move waypoints* and *non-Vision Move waypoints*.
- **Batching:** default **max 20** poses per reply; configurable in *Robot Communication Configuration → Next → Advanced Settings*, **hard limit 30**. Default reply **timeout 10 s** (same Advanced Settings panel).
- **Robot-side buffer:** *"The robot can obtain up to 100 vision points or waypoints from the vision system."* ([FANUC](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/fanuc-interface-commands.html), same sentence on ABB/KUKA/YASKAWA pages.)

### 1.2 Mech-Vision-related commands (2.2.1)

| # | Official name | Sent (client → vision) | Reply (vision → client) | Success code |
|---|---|---|---|---|
| **100** | Trigger Mech-Vision Project and Get Results | `100, Mech-Vision project ID, parameter recipe ID, returned data format, robot joint positions, robot flange pose` | depends on *returned data format* 1–4, see §1.5 | **1100** |
| **101** | Trigger Mech-Vision Project | `101, Mech-Vision project ID, expected number of vision points or waypoints, robot pose type, robot pose` | `101, status code` | **1102** |
| **102** | Get Vision Results | `102, Mech-Vision project ID` | `102, status code, status of transmitting vision points, number of vision points, reserved field, vision point 1 (TCP, label, tool ID), vision point 2 (…), …` | **1100** |
| **103** | Switch Mech-Vision Parameter Recipe | `103, Mech-Vision project ID, parameter recipe ID` (recipe ID = positive int 1–99) | `103, status code` | **1107** |
| **104** | Switch Mech-Vision Solution | `104, Mech-Vision Solution ID` | `104, status code` | **1104** |
| **105** | Get Planned Path from Mech-Vision | `105, Mech-Vision project ID, waypoint pose type` (1 = JPs, 2 = tool pose) | `105, status code, status of transmitting waypoints, number of waypoints, position of "Vision Move" in planned path, waypoint 1 (pose, label, tool ID), …` | **1103** |
| **106** | Get Gripper DO List from Mech-Vision | `106, Mech-Vision project ID, number of vacuum gripper sections` | `106, status code, DO signal 1 … DO signal 64` | **1106** |
| **110** | Get Custom Data from Mech-Vision | `110, Mech-Vision project ID` | `110, status code, status of transmitting vision points, number of elements in custom data (N), pose, label, element 1 … element N` | **1100** |
| **111** | Get Vision Move Data from Mech-Vision | `111, Mech-Vision project ID, waypoint pose type` | `111, status code, status of transmitting waypoints, waypoint type, pose, motion type, tool ID, Vision Move data` | **1103** |
| **501** | Input Object Dimensions to Mech-Vision Project | `501, Mech-Vision project ID, length, width, height` (mm) | `501, status code` | **1108** |
| **503** | Input Poses to Mech-Vision Project | `503, Mech-Vision project ID, Step name, pose` | `503, status code` | **1110** |
| **504** | Set Numeric Global Variable | `504, Global Variable ID, Global Variable Value` | `504, status code` | **1111** |
| **505** | Get Numeric Global Variable | `505, Global Variable ID` | `505, status code, Global Variable Value` | **1111** |
| **506** | Set String Global Variable | `506, Global Variable ID, Global Variable Value` | `506, status code` | **1111** |
| **507** | Get String Global Variable | `507, Global Variable ID` | `507, status code, Global Variable Value` | **1111** |
| **601** | Get Message from Notify Step | `601` | `601, message from Notify Step` | *(no status code — returns the Message value directly)* |
| **701** | Calibration | `701, calibration status, flange pose, joint positions` | `701, status code, calibration status, flange pose of next calibration point, joint positions of next calibration point` | **7101** |
| **901** | Get System Status | `901` | `901, status code, Mech-Vision Status, Mech-Viz Status, Camera Status` | **9100** |

**Command 101 `robot pose type` table (verbatim):**

| Type | Robot pose sent | Applicable scenario |
|---|---|---|
| 0 | `0,0,0,0,0,0` — no pose sent; Path Planning start point = Home point in the path planning tool | eye-to-hand, no pre-capture required |
| 1 | Robot's current JPs **and** flange pose | eye-in-hand; recommended for most scenarios except gantry robots |
| 2 | Robot's current flange pose | recommended for gantry robots |
| 3 | User-defined joint positions (used as Path Planning start point) | eye-to-hand where the project requires images captured beforehand |

**Command 901 status decode:** Mech-Vision Status 0 = not opened / 1 = opened; Mech-Viz Status 0 = not opened / 1 = opened; Camera Status 0 = at least one camera disconnected / 1 = all cameras connected normally.

**Command 701 calibration handshake:** sent `calibration status` 0 = notify Mech-Vision to initiate calibration, 1 = robot reached previous calibration point, 2 = robot failed to reach previous calibration point. Returned `calibration status` 0 = in progress, 1 = calibration finished. Used together with the **Camera Calibration** tool on the Mech-Vision toolbar.

### 1.3 Mech-Viz-related commands (2.2.1)

| # | Official name | Sent | Reply | Success code |
|---|---|---|---|---|
| **200** | Trigger Mech-Viz Project and Get Planned Path | `200, Branch by Msg Step ID, exit port, waypoint pose type, robot joint positions, robot flange pose` | `200, status code, status of transmitting waypoints, number of waypoints, position of "Vision Move" in planned path, waypoint 1 (pose, label, tool ID), …` | **2100** |
| **201** | Trigger Mech-Viz Project | `201, robot pose type, robot pose` (pose type 0–2) | `201, status code` | **2103** |
| **202** | Stop Mech-Viz Project | `202` | `202, status code` | **2104** |
| **203** | Set Exit Port for Branch by Msg in Mech-Viz | `203, Step ID, exit port` | `203, status code` | **2105** |
| **204** | Set Current Index for Mech-Viz | `204, Step ID, Current Index` | `204, status code` | **2106** |
| **205** | Get Planned Path from Mech-Viz | `205, waypoint pose type` (1 = JPs, 2 = tool pose) | `205, status code, status of transmitting waypoints, number of waypoints, position of "Vision Move" in planned path, waypoint 1 (pose, label, tool ID), …` | **2100** |
| **206** | Get Gripper DO List from Mech-Viz | `206, number of vacuum gripper sections` | `206, status code, DO signal 1 … DO signal 64` | **2102** |
| **207** | Read Mech-Viz Step Parameter | `207, Config ID` | `207, status code, Step parameter value` | **2109** |
| **208** | Set Mech-Viz Step Parameter | `208, Config ID` | `208, status code` | **2108** |
| **210** | Get Vision Move Data or Custom Data from Mech-Viz | `210, returned data format` (1–4) | `210, status code, status of transmitting waypoints, waypoint type, pose, motion type, tool ID, velocity, Vision Move data, number of elements in custom data (N), element 1 … element N` | **2100** |
| **601** | Get Message from Notify Step | `601` | `601, message from Notify Step` | — |

**Command 201 `robot pose type`:** 0 = `[0,0,0,0,0,0]`, simulated robot moves from its set home position (eye-to-hand); 1 = current JPs + flange pose (recommended eye-in-hand); 2 = JPs customized by the robot side, i.e. a taught image-capture point (recommended eye-to-hand, allows planning ahead and shortens cycle time). Docs state explicitly: *"robot pose type should be set to 2 for projects in eye to hand mode."*

**Off-by-one traps (both verbatim from docs):**
- 203 / 200 `exit port`: *"When the parameter value is set to 'N', the Mech-Viz project exits from the port with an ID of 'N-1'."* `203, 2, 1` → port **0** of Step 2.
- 204 `Current Index`: *"When this parameter value is set to 'N', the Current Index of the corresponding Step is 'N-1'."* `204, 2, 1` → index **0**.

**207 / 208 `Config ID`** is defined in a `property_config` file (Mech-Vision toolbar → *Robot Communication Configuration → Next → Advanced Settings → Property Configuration*):
```
read,  <Config ID>, <Step ID>, <parameter key name>
write, <Config ID>, <Step ID>, <parameter key name>, <parameter value>
```
Example lines: `read, 5, 3, xCount` and `write, 6, 3, xOffset, 0.000000`. Multiple `write` lines may share one Config ID (sets several parameters in one command); `read` Config IDs must be unique. Key names come from Mech-Viz *Tools → Key Query Tool* (requires Developer mode). **The interface service must be restarted after editing `property_config`.**

**Vision Move data payload (identical for 111 and 210):**

| Field | Elements |
|---|---|
| Labels of picked workobjects (10 integers, default ten 0s) | 10 |
| Number of picked workobjects | 1 |
| Number of workobjects to be picked this time | 1 |
| Edge/corner ID of vacuum gripper | 1 |
| TCP offset (XYZ between workobject-group center and tool pose center) | 3 |
| Orientation of workobject group (0 = parallel, 1 = vertical) | 1 |
| Orientation of workobject (0 = parallel, 1 = vertical) | 1 |
| Dimensions of workobject group (L, W, H) | 3 |

`motion type`: **1 = joint motion (MOVEJ), 2 = linear motion (MOVEL)**. `tool ID` = -1 means no tool used at that waypoint.

### 1.4 Legacy (1.7.4) names — for older sites

Source: [TCP/IP Interface Commands (1.7.4)](https://docs.mech-mind.net/en/robot-integration/1.7.4/standard-interface-development-manual/tcp-ip-socket.html)

| # | 1.7.4 official name | 2.2.1 official name |
|---|---|---|
| 101 | Start Mech-Vision Project | Trigger Mech-Vision Project |
| 102 | Get Vision Target(s) | Get Vision Results |
| 103 | Switch Mech-Vision Recipe | Switch Mech-Vision Parameter Recipe |
| 105 | Get Result of Step "Path Planning" in Mech-Vision | Get Planned Path from Mech-Vision |
| 110 | Get Custom Output Data from Mech-Vision | Get Custom Data from Mech-Vision |
| 201 | Start Mech-Viz Project | Trigger Mech-Viz Project |
| 202 | Stop Mech-Viz Project | (same) |
| 203 | Select Mech-Viz Branch | Set Exit Port for Branch by Msg in Mech-Viz |
| 204 | Set Move Index | Set Current Index for Mech-Viz |
| 205 | Get Planned Path | Get Planned Path from Mech-Viz |
| 206 | Get DO List | Get Gripper DO List from Mech-Viz |
| 207/208 | Read / Set Mech-Viz Step Parameter | (same) |
| 210 | Get Waypoint with Depalletizing Planning Data | Get Vision Move Data or Custom Data from Mech-Viz |
| 501 | Input Object Dimensions to Mech-Vision | Input Object Dimensions to Mech-Vision Project |
| **502** | **Input TCP to Mech-Viz** — `502, robot TCP`; reply `502, status code` (**2107**); read by the **External Move** Step; template project at `Mech-Center\tool\viz_project\outer_move`; must be called before 201 | **removed / not present in the 2.2.1 TCP command list** |
| 601 | Notify | Get Message from Notify Step |
| 701 | Calibration | (same) |
| **901** | **Get Software Status** — reply `901, status code`; **1101** = Mech-Vision ready to run projects; anything else = not ready | Get System Status (now returns 3 status fields, success **9100**) |

Commands **100, 104, 106, 111, 200, 503, 504, 505, 506, 507** do **not** exist in 1.7.4 — they are 2.x additions.

> **Correction to a common assumption:** there is **no** "Command 501 = get software status". 501 is *Input Object Dimensions*; software/system status is **901**.

### 1.5 Command 100 `returned data format` (2.2.1)

| Value | Returned data |
|---|---|
| 1 | `100, status code, status of transmitting vision points, number of vision points, reserved field, vision point 1 (TCP, label, tool ID), vision point 2 (…), …` |
| 2 | `100, status code, status of transmitting vision points, number of elements in custom data (N), pose, label, element 1 … element N` |
| 3 | `100, status code, status of transmitting waypoints, number of waypoints, position of "Vision Move" in planned path, waypoint 1 (joint positions, label, tool ID), …` |
| 4 | same as 3 but waypoints as **TCP** |

Real example pair from the docs:
```
→ 100, 1, 2, 1, 5.18, 14.52, 4.03, 0.09, 72.44, 5.15, 549.56, 50.0, 647.01, 180.0, -1.0, 180.0
← 100, 1100, 1, 1, 0, 95.7806, 644.5677, 401.1013, 91.1206, -171.1301, 180.0, 0, 0
```

### 1.6 Calling sequence (canonical)

Source: [Calling Sequence of Standard Interface Commands](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-development-manual/control-timing.html)

**Mech-Vision chain:** `103` and/or `501` (before) → **`101`** → one of `102` / `105` / `110`. *"Command 102, Command 105, and Command 110 cannot be used at the same time."*

**Mech-Viz chain:** `207` / `208` (before) → **`201`** → `204` then `203` (204 before 203) → `205` **or** `210` → after the project stops, `206`. *"Command 205 and Command 210 cannot be used at the same time."* For index-type Steps the docs prescribe the order **201 → 204 → 203** *"to ensure that Mech-Viz has enough time to set the Current Index value."*

Other stated ordering constraints: `104` before `100`/`101`; `106` after `100`/`105`/`111`; `601` *"immediately AFTER Command 101 or Command 201"*; `503` before `101`.

**PLC handshake (PROFINET/EtherNet/IP/Snap7/MC/Modbus)** — verbatim sequence: write command number + parameters to registers → PLC sets `Command_Trigger = 1` → vision reads → vision sets `Trigger_Acknowledge = 1` → PLC sets `Command_Trigger = 0` → vision sets `Trigger_Acknowledge = 0`.

---

## 2. Status / error codes

Source: [Status Codes and Troubleshooting (2.2.1)](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-development-manual/status-codes-error-troubleshooting.html)

### 2.1 Ranges (verbatim)

| Range | Category |
|---|---|
| 1001–1099 | Mech-Vision error codes |
| 1100–1199 | Mech-Vision normal status codes |
| 2001–2099 | Mech-Viz error codes |
| 2100–2199 | Mech-Viz normal status codes |
| 3001–3099 | Communication Component error codes |
| 3100–3199 | Communication Component normal status codes |
| 4001–4099 | Robot error codes |
| 4100–4199 | Robot normal status codes |
| 6001~6199 | **Status codes that can be customized in Mech-Vision** |
| 7001–7099 | Calibration error codes |
| 7100–7199 | Calibration normal status codes |

In 1.6/1.7 the 3xxx block was labelled **Mech-Center**; in 2.x it is **Communication Component** (Mech-Center no longer exists as a separate service). There is **no 5xxx block** in the Standard Interface code space — flagged because 5xxx is sometimes assumed. **UNVERIFIED / not found:** any documented 5xxx Standard Interface status range.

### 2.2 Mech-Vision error codes (1001–1099)

| Code | Description |
|---|---|
| 1001 | Mech-Vision: Unregistered project existed in the solution |
| 1002 | Mech-Vision: No vision result |
| 1003 | Mech-Vision: No point cloud in ROI |
| 1005 | Mech-Vision: Invalid command parameter to start Mech-Vision project |
| 1006 | Mech-Vision: Invalid pose data |
| 1007 | Mech-Vision: Project is running |
| 1008 | Mech-Vision: Digital output signal list not provided |
| 1010 | Mech-Vision: Number of poses and number of labels do not match |
| 1011 | Mech-Vision: Project ID does not exist |
| 1012 | Mech-Vision: Parameter recipe ID does not exist |
| 1013 | Mech-Vision: Parameter recipe not configured |
| 1014 | Mech-Vision: Failed to switch parameter recipe |
| 1015 | Mech-Vision: Project execution error |
| 1016 | Mech-Vision: Data obtained by the command does not match the Port Type value of the Output Step. Please select a proper command or modify the Port Type value |
| 1017 | Mech-Vision: Failed to map the string of label to numbers |
| 1018 | Mech-Vision: Invalid pose number input |
| 1019 | Mech-Vision: Execution timeout |
| 1020 | Mech-Vision: Project not started |
| 1021 | Mech-Vision: Failed to set object dimensions |
| 1022 | Mech-Vision: Invalid object dimension settings |
| 1023 | Mech-Vision: Failed to connect to camera |
| 1024 | Mech-Vision: Number of poses and list size of custom port data do not match |
| 1025 | Mech-Vision: Visual image data in use. The running cannot proceed |
| 1026 | Mech-Vision: Invalid pose type |
| 1027 | Mech-Vision: Runtime error at Path Planning Step |
| 1030 | Mech-Vision: Robot cannot reach the waypoint |
| 1033 | Mech-Vision: Motion singularity error |
| 1035 | Mech-Vision: Invalid pick point |
| 1036 | Mech-Vision: Robot collision detected |
| 1037 | Mech-Vision: No available placing position for palletizing |
| 1044 | Mech-Vision: No vision point for the Vision Move Step |
| 1046 | Mech-Vision: Invalid tool |
| 1047 | Mech-Vision: Timeout occurs when waiting for the capture completion |
| 1048 | Mech-Vision: Box mask recognition error |
| 1049 | Mech-Vision: Check box dimensions error |
| 1051 | Mech-Vision: Failed to set pose |
| 1052 | Mech-Vision: Failed to declare/get global variable |
| 1053 | Mech-Vision: Failed to execute the switch solution instruction |

### 2.3 Mech-Vision normal status codes (1100–1199)

| Code | Description |
|---|---|
| 1100 | Mech-Vision: Get vision result successfully |
| 1101 | Mech-Vision: Ready to run |
| 1102 | Mech-Vision: Project get triggered successfully |
| 1103 | Mech-Vision: Obtained planned path successfully |
| 1106 | Mech-Vision: Obtain DO signal list successfully |
| 1107 | Mech-Vision: Parameter recipe switched successfully |
| 1108 | Mech-Vision: Object dimensions input to the project successfully |
| 1110 | Mech-Vision: Pose set successfully |

*(1104 = solution switched successfully and 1111 = global variable set/read successfully are used in the Command 104 / 504–507 pages but are **not listed** in the 2.2.1 status-code table — **inconsistency in the official docs**, flagged.)*

### 2.4 Mech-Viz error codes (2001–2099)

| Code | Description |
|---|---|
| 2001 | Mech-Viz: Software not registered |
| 2002 | Mech-Viz: Project is running |
| 2004 | Mech-Viz: Robot cannot reach the waypoint |
| 2006 | Mech-Viz: Invalid command parameter to start Mech-Viz |
| 2008 | Mech-Viz: Runtime error |
| 2011 | Mech-Viz: Digital output signal list not provided |
| 2012 | Mech-Viz: Invalid pose type |
| 2013 | Mech-Viz: Invalid pose data |
| 2014 | Mech-Viz: No project set to autoload |
| 2015 | Mech-Viz: Project not opened |
| 2016 | Mech-Viz: Failed to set Step parameter value |
| 2017 | Mech-Viz: Failed to stop execution |
| 2018 | Mech-Viz: Invalid branch exit port number |
| 2019 | Mech-Viz: Failed to set branch |
| 2020 | Mech-Viz: Motion singularity error |
| 2022 | Mech-Viz: The project has not been executed or has no results after execution |
| 2024 | Mech-Viz: Invalid Branch Step ID |
| 2025 | Mech-Viz: Execution timeout |
| 2026 | Mech-Viz: Invalid Step ID of index-type Steps |
| 2027 | Mech-Viz: Invalid Current Index value |
| 2028 | Mech-Viz: Failed to set index |
| 2030 | Mech-Viz: Invalid pick point |
| 2031 | Mech-Viz: Robot collision detected |
| 2032 | Mech-Viz: No available placing position for palletizing |
| 2036 | Mech-Viz: Visual Recognition Step not called |
| 2037 | Mech-Viz: No vision result received from vision service |
| 2038 | Mech-Viz: No point cloud in ROI |
| 2039 | Mech-Viz: No vision point for the Vision Move Step |
| 2041 | Mech-Viz: Failed to get Step parameter |
| 2042 | Mech-Viz: Failed to get planning result in Vision Move |
| 2043 | Mech-Viz: Failed to get custom data |
| 2044 | Mech-Viz: Vision service not registered |
| 2045 | Mech-Viz: Invalid tool |
| 2047 | Mech-Viz: Check box dimensions error |

### 2.5 Mech-Viz normal status codes (2100–2199)

| Code | Description |
|---|---|
| 2100 | Mech-Viz: Execution completed successfully |
| 2102 | Mech-Viz: DO signal list obtained successfully |
| 2103 | Mech-Viz: Started successfully |
| 2104 | Mech-Viz: Stopped successfully |
| 2105 | Mech-Viz: Set branch successfully |
| 2106 | Mech-Viz: Set index successfully |
| 2107 | Mech-Viz: Set move point for External Move Step successfully |
| 2108 | Mech-Viz: Set Step parameter value successfully |
| 2109 | Mech-Viz: Get Step parameter value successfully |

### 2.6 Communication Component, Robot, Calibration

| Code | Description |
|---|---|
| 3001 | Communication Component: Invalid command |
| 3002 | Communication Component: Invalid data length or format for command parameter |
| 3005 | Communication Component: Mech-Vision timed out *(vision system timed out calling the gRPC service)* |
| 3006 | Communication Component: Unknown error |
| 3007 | Communication Component: Data acknowledge signal from client timed out |
| 3008 | Communication Component: Configuration ID does not exist |
| 3103 | Communication Component: Modbus TCP data cleared successfully *(only normal 3xxx code in 2.2.1)* |
| 4002 | Robot: Euler angle type not supported *(only 4xxx code in 2.2.1)* |
| 7001 | Calibration: Parameter error |
| 7002 | Calibration: No calibration flange pose provided from Mech-Vision |
| 7003 | Calibration: Calibration joint positions not provided by Mech-Vision |
| 7004 | Calibration: Robot failed to reach the calibration point |
| 7100 | Calibration: Robot moved to the calibration point successfully |
| 7101 | Calibration: Pose received from Mech-Vision successfully |

Legacy 1.6 3xxx codes that were **dropped** in 2.x but still appear in field notes: `3003` Client disconnected, `3004` Server disconnected, `3100` Client connection normal, `3101` Server connection normal, `3102` Waiting for client to connect ([1.6 status codes](https://docs.mech-mind.net/1.6/en-GB/SoftwareSuite/MechCenter/MechInterface/StandardInterfaceDevelopmentManual/StatusCodesAndErrorTroubleshooting/StatusCodesAndErrorTroubleshooting.html)).

**3007 detail (2.2.1, verbatim causes)** — fires when, on PROFINET/EtherNet/IP, the client fails within the timeout (default 10 s) to: reset `Data_Acknowledge` to 0 before new pose data is sent; set `Data_Acknowledge` to 1 after pose data is sent; clear `NOTIFY` via `CLEAR_NOTIFY` before the next notify message (Snap7/Modbus TCP/MC/PROFINET/EtherNet/IP); or reset `EXPOSURE_COMPLETE` via `RESET_EXPOSURE` before the next exposure.

### 2.7 `3099` — socket open failure

**`3099` is used in every official example program** ("`status=3099` means failed to open socket", e.g. [FANUC MM_S1_Vis_Basic](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/fanuc-s1-vis-basic.html); ABB `MM_S1_Vis_Basic` tests `IF status = 3099 THEN TPWrite "MM: Communication Error"`), **but 3099 is not listed in the official status-code table.** Treat "3099 = failed to open socket / communication error" as **documented-by-example only**.

### 2.8 `CV-Exxxx` and `error_code` / `exit_code`

- **`CV-Exxxx` is a Mech-Vision *internal algorithm/Step* error code, not a Standard Interface status code.** The 1.6 troubleshooting text for 1015 states verbatim: *"The Mech-Vision project has runtime error with error code CV-Exxxx or has other errors for which Mech-Center does not have analyses. When such an error occurs, the Mech-Vision project is terminated without finishing execution."* The Standard Interface surfaces this only as **1015** ("Project execution error"); the 2.2.1 note adds *"This status code only indicates that an error has occurred… Specific reasons for the error was not provided."* The `CV-Exxxx` text appears in the Mech-Vision **Log panel → Console tab**, not on the wire.
- Analogous prefix: `DL-E0201 deep learning server not started` maps to status **1016**.
- **`CV-E0000` specifically: UNVERIFIED.** I could not find a page enumerating `CV-Exxxx` values or a `CV-E0000` entry on docs.mech-mind.net. Do not publish a meaning for it.
- **`exit_code`: UNVERIFIED.** No `exit_code` field exists anywhere in the Standard Interface reply formats or status-code documentation. If a project of yours uses `exit_code`, it is either an Adapter-layer custom field or a Mech-Vision Step output, not Standard Interface.
- Where errors are visible operationally: [Error Code Processing](https://docs.mech-mind.net/en/suite-service-manual/latest/troubleshooting/troubshooting-common-issues-error-codes.html) — *"logs with error codes will be printed in the Console tab of the Mech-Vision's Log panel, such as '[1019]: Mech-VisionExecution timeout'."*

---

## 3. Structure of the Command 102 vision result

`102, status code, status of transmitting vision points, number of vision points, reserved field, vision point 1 (TCP, label, tool ID), vision point 2 (…), …`

**Per-field semantics (verbatim):**

- **status code** — 1100 on success.
- **status of transmitting vision points** — 0 = not all vision points obtained; 1 = all obtained. *"If the value is 0, repeat sending the command until the value becomes 1. If not all vision points are obtained when Command 101 is called, the remaining vision points will be cleared."*
- **number of vision points** — number returned in *this* reply. Default max 20 per reply (limit 30).
- **reserved field** — *"This field is not currently in use. The value is 0."*
- **vision point** — exactly **8 elements**: elements 1–6 = TCP, element 7 = label, element 8 = tool ID.

**Pose format — the important part:**

- On the wire it is **always 6-DoF Euler, never a quaternion**: `X Y Z` in **mm**, `A B C` as **Euler angles in degrees**, with the Euler convention set by the selected robot brand in the communication configuration.
- Mech-Vision internally holds object poses as **quaternions** (the `poses` port of the **Output** Step emits `[x, y, z, qw, qx, qy, qz]` — 7 columns, as shown in the Command 110 port table). The vision system performs two conversions before transmitting:
  1. *"Convert the object pose from the form of quaternions to Euler angles."*
  2. *"Rotate the object's pose around the X-axis by 180° to orient its Z-axis downward."*
- Also verbatim: *"If the first input port of the Output Step is **Object Center Points**, the Output Step will convert the object center points into the corresponding **pick points**. Therefore, the object poses obtained by running this command are actually poses of pick points, instead of poses of object center points."*
- **Label** — *"The label must be an integer-formatted string. If no label information is available, the label value defaults to 0."* (String labels must be mapped to integers; failure = **1017**.)
- **Tool ID** — *"The default value of this parameter is 0. In most cases, vision points output by Mech-Vision do not have information of the object tool ID."* For **waypoints** (105/205) the tool ID is the one set in the path planning tool / Mech-Viz project.

**Per-target custom data** is *not* available through 102 — use **110**. Structure: `110, status code, status of transmitting vision points, number of elements in custom data (N), pose, label, element 1 … element N`. Key constraints (verbatim):
- *"Each time the robot executes this command, it only obtains the pose and custom port output corresponding to **one** vision point… To obtain the custom data corresponding to multiple vision points, call this command multiple times."*
- N = sum of the **column counts** of all custom ports (e.g. `customData1` with 3 columns + `customData2` with 2 columns → N = 5).
- *"The custom port outputs are arranged in the **alphabetical order** of the names of custom ports."*
- Port Type of the Output Step must be **Custom**, and the `poses` port is required.

**Batching ("how many poses per request"):**
- The client declares its appetite in **101** via `expected number of vision points or waypoints` (0 = all).
- The server caps each reply at **20 poses by default, 30 maximum** (*Robot Communication Configuration → Next → Advanced Settings → maximum number of poses to obtain each time*).
- The client loops on **102** until `status of transmitting vision points == 1`. Documented 22-point example:
  ```
  → 101, 1, 0, 1, -0, -20.6323, -107.8121, -0, -92.8181, 0.0016
  ← 101, 1102
  → 102, 1        ← 102, 1100, 0, 20, 0, 95.7806, 644.5677, 401.1013, 31.1206, …
  → 102, 1        ← 102, 1100, 1,  2, 0, 315.2017, 592.1261, 399.6052, 126.1960, …
  ```
- Reply timeout 10 s default, per-request, configurable.
- For **105/205**, note the documented semantic shift of `position of "Vision Move" in planned path` across multiple replies: *"In the first response, it indicates the position of the Vision Move waypoint in the entire path, while in subsequent responses, this parameter indicates the position of the Vision Move waypoint among the **remaining** waypoints."* Also: set 101's expected-count to **0** before using 105, otherwise you must call 105 once per waypoint.

> **Terminology correction:** there is no parameter or concept named **`sendPose`** in Mech-Mind documentation. The robot-side parameter is **`SendPos_Type`** (YASKAWA: `sendpos_type`) and it selects *which robot pose is sent to the vision system* (0–3), not batching. Batching is governed by `Pos_Num_Need` + the Advanced Settings pose cap + the transmit-status loop.

---

## 4. Robot-side program structure

All four brands follow the same shape: a **background/driver module** (socket + protocol) plus **thin wrapper routines named `MM_*`** called from the user's motion program; results land in **registers/variables**, never as return values. **Two-stage retrieval is mandatory on all brands:** a *Get* command fetches data into robot memory, then a *Store* command copies one indexed item into a position register / variable. FANUC docs state this explicitly: *"The returned vision result is saved to the robot memory and cannot be directly obtained. To access the vision result, you must store the vision result in a subsequent step."*

Program folder on the IPC (all brands): `Communication Component/Robot_Interface/<BRAND>/` in the Mech-Vision & Mech-Viz install directory, reachable via *Robot Communication Configuration → Robot integration → Open program folder*. Example programs live in `.../<BRAND>/sample/`.

### 4.1 FANUC

Source: [FANUC Standard Interface Commands](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/fanuc-interface-commands.html) · [Setup](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/fanuc-setup-instructions.html) · [Example programs](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/fanuc-example-program.html)

**Prerequisites (verbatim):** controller system software V7.5, 7.7, 8.x, 9.x or 10.1. **`R651` or `R632` (KAREL) MUST be installed. `R648` (User Socket Msg) MUST be installed.** Ethernet to controller motherboard **CD38A (= Port 1)** or **CD38B (= Port 2)**.

**Loading:** copy *the contents* of the `FANUC` folder (not the folder itself) to the **root** of a ≤32 GB FAT32 flash drive, then on the TP: `MENU → FILE → F5 UTIL → Set Device → USB Disk (UD1:)` or `USB on TP (UT1:)`, select **`INSTALL`**, `ENTER`, `F4 YES`. Success message: **"Programs Loaded"**. Communication test program: **`MM_COMTEST`** (4 args: robot port 1–8, IPC IP, IPC port, timeout min). *Changing the robot port number for the first time requires a controller restart ("MM:Restart Robot").*
> **UNVERIFIED:** the setup page does not enumerate the individual `.pc` / `.tp` / `.vr` files installed by `INSTALL`; do not publish a FANUC file list.

| Feature | Routine (exact signature) |
|---|---|
| Communication init | `MM_INIT_SKT(C_Tag, Ip_Addr, Svr_Port, Time_Out)` — `C_Tag` = **string** robot port `'1'`–`'8'`; `Time_Out` in **minutes**; V10: `Svr_Port` 1–32767 |
| Run Mech-Vision project | `MM_START_VIS(Job, Pos_Num_Need, SendPos_Type, Pr_Num, MM_Status)` |
| Get vision result | `MM_GET_VIS(Job, Reg_Pos_Num, MM_Status)` |
| Store result/path (TCP) | `MM_GET_POS(Serial, Pr_Num, Reg_Label, Reg_ToolId)` — `Serial` is **1-based** |
| Store path (joint positions) | `MM_GET_JPS(Serial, Pr_Num, Reg_Label, Reg_ToolId)` |
| Switch parameter recipe | `MM_SET_MOD(Job, Model_Num, MM_Status)` |
| Get planned path (Mech-Vision) | `MM_GET_VISP(Job, Jps_Pos, Reg_Pos_Num, Reg_VPos_Num, MM_Status)` |
| Get Mech-Vision custom data | `MM_GET_DY_DT(Job, Reg_Pos_Num, MM_Status)` |
| Store Mech-Vision custom data | `MM_GET_DYPOS(Serial, Pr_Num, Reg_Label, Reg_UserData)` |
| Get gripper DO list | `MM_GET_DL(Resource, Block_Num, MM_Status)` |
| Set gripper DO list | `MM_SET_DL(Loop_Index)` |
| Run Mech-Viz project | `MM_START_VIZ(SendPos_Type, Pr_Num, MM_Status)` |
| Set branch exit port | `MM_SET_BCH(Branch_Num, Export_Num, MM_Status)` |
| Set current index | `MM_SET_IDX(Skill_Num, Index_Num, MM_Status)` |
| Get planned path (Mech-Viz) | `MM_GET_VIZ(Jps_Pos, Reg_Pos_Num, Reg_VPos_Num, MM_Status)` |
| Get Vision Move / custom data | `MM_GET_PLNDT(Resource, Jps_Pos, Reg_Pos_Num, Reg_VPos_Num, MM_Status)` |
| Store Vision Move / custom data | `MM_GET_PLJOP(Serial, Jps_Pos, Pr_Num, Reg_MoveType, Reg_ToolNum, Reg_Speed, Reg_UserData, Reg_PlanRes)` |
| Read Mech-Viz Step parameter | `MM_GET_PROP(Read_id, MM_Status, Reg_Viz_Prop)` |
| Set Mech-Viz Step parameter | `MM_SET_PROP(Write_id, MM_Status)` |
| Input object dimensions | `MM_SET_BS(Job, Length, Width, Height, MM_Status)` |
| Input pose data | `MM_SET_VISP(Job, Step_Name, Pr_Num, MM_Status)` |
| Get Notify message | `MM_GET_NTFY(MM_NotifyMsg)` |
| Calibration | `MM_CALIB(Move_Type, PosJps, WaitTime, AxisNum, AxisVal, Reg_CalibPos, Reg_CalibMov)` |
| Stop Mech-Viz project | `MM_Stop_Viz(MM_Status)` |
| Get system status | `MM_GET_STAT(MM_Status)` |

**Canonical FANUC TP flow** (`MM_S1_Vis_Basic`, file at `Communication Component/Robot_Interface/FANUC/sample/MM_S1_Vis_Basic`):
```
UFRAME_NUM=0 ; UTOOL_NUM=1 ;
J P[1] 100% FINE ;                            ! home
CALL MM_INIT_SKT('8','127.0.0.1',30000,5) ;   ! once only
L P[2] 1000mm/sec FINE ;                      ! image-capture pose
CALL MM_START_VIS(1,0,2,10,53) ;
IF (R[53]<>1102),JMP LBL[99] ;
CALL MM_GET_VIS(1,51,53) ;
IF R[53]<>1100,JMP LBL[99] ;
CALL MM_GET_POS(1,60,70,80) ;                 ! PR[60]=TCP, R[70]=label, R[80]=toolId
J P[3] 50% CNT100 ;
L PR[60] 1000mm/sec FINE Tool_Offset,PR[1] ;  ! approach
L PR[60]  300mm/sec FINE ;                    ! pick
...
LBL[99:vision error] ;                        ! 1003 no cloud in ROI / 1002 no result / 3099 socket
```
FANUC has **21+ documented example programs** `MM_S1_Vis_Basic` … `MM_S22_Vis_As_Uframe`, including `MM_S6_Viz_ErrorHandle`, `MM_S8/S10_Viz_Subtask`, `MM_S9_Viz_RunInAdvance` (pipelined capture to cut cycle time), `MM_S13_Vis_MoveInAdvance`, `MM_S15_Viz_GetDoList`, `MM_S19/S20_PlanAllVision`, `MM_S21_1Robot_3IPC_Sequentially`.

### 4.2 ABB (RAPID)

Source: [ABB Standard Interface Commands](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/abb-interface-commands.html) · [Setup (RobotWare 6)](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/abb-setup-instructions.html)

**Prerequisites:** controller IRC4 or IRC5; **RobotWare 6.02–6.15**; control module **616-1 PC Interface** installed. Ethernet to controller **X6 (WAN)** port.

**Files loaded to task `T_ROB1`** (verbatim, from `.../ABB/RobotWare 6/`):
- **`MM_Module.mod`** — program module file
- **`MM_Auto_Calib.mod`** — calibration program module file
- **`MM_Com_Test.mod`** — communication-testing program module file

**Version dependency:** *"For RobotWare6, the file extension is `.mod`. For RobotWare7, please modify the file extension from `.mod` to `.modx`."* Auto-loading is available via `Communication Component\tool\Robot Program Loader\Robot Program Loader` (back up first, then *Load the Standard Interface program* → *Load with one-click*); manual loading via TP `Program Editor → Tasks and Programs → T_ROB1 → Show Modules → File → Load Module…` or RobotStudio *Load Module*.

ABB is the only brand with **explicit open/close** socket routines.

| Feature | Routine (exact signature) |
|---|---|
| Initialize communication | `MM_Init_Socket IP_Address, Server_Port, Time_Out;` — `Time_Out` in **seconds** |
| Establish TCP communication | `MM_Open_Socket;` |
| Close TCP communication | `MM_Close_Socket;` |
| Run Mech-Vision project | `MM_Start_Vis Job, Pos_Num_Need, SendPos_Type, MM_J, MM_Status;` |
| Get vision result | `MM_Get_VisData Job, Pose_Num, MM_Status;` |
| Store result/path (TCP) | `MM_Get_Pose Serial, MM_P, MM_Label, MM_ToolId;` |
| Store path (joint positions) | `MM_Get_Jps Serial, MM_J, MM_Label, MM_ToolId;` |
| Switch parameter recipe | `MM_Switch_Model Job, Model_Number, MM_Status;` |
| Get planned path (Mech-Vision) | `MM_Get_VisPath Job, Jps_Pos, Pos_Num, VisPos_Num, MM_Status;` |
| Trigger + get result (one shot) | `MM_Lite_Vis Job, Model_Number, Recv_Data_Type, MM_Status, \MM_J, \Pos_Num, \VisPos_Num;` — **max 20 poses** |
| Get Mech-Vision custom data | `MM_Get_DyData job, Pos_Num, MM_Status;` |
| Store Mech-Vision custom data | `MM_Get_DyPose Serial, MM_P, MM_Label;` |
| Get gripper DO list | `MM_Get_DoList Resource, BlockNum, MM_Status;` |
| Set gripper DO list | `MM_Set_DoList Loop_Index, Serial, Go16;` |
| Run Mech-Viz project | `MM_Start_Viz SendPos_Type, MM_J, MM_Status;` |
| Set branch exit port | `MM_Set_Branch Branch_Num, Exit_Num, MM_Status;` |
| Set current index | `MM_Set_Index Skill_Num, Index_Num, MM_Status;` |
| Get planned path (Mech-Viz) | `MM_Get_VizData Jps_Pos, Pos_Num, VisPos_Num, MM_Status;` |
| Trigger Mech-Viz + get path | `MM_Lite_Viz Branch_Num, Export_Num, Recv_Data_Type, MM_Status, \Pos_Num, \VisPos_Num;` |
| Get Vision Move / custom data | `MM_get_plandata Resource, Jps_Pos, Pos_Num, VisPos_Num, MM_Status;` |
| Store Vision Move / custom data | `MM_Get_PlanPose Serial, Jps_Pos, MM_P, MM_MoveType, MM_ToolNum, MM_Speed;` *(when `Jps_Pos` is 2 or 4)* |
| Read Mech-Viz Step parameter | `MM_Get_Property Get_Id, MM_Status, MM_Viz_Prop;` |
| Set Mech-Viz Step parameter | `MM_Set_Property Set_Id, MM_Status;` |
| Input object dimensions | `MM_Set_BoxSize Job, Length, Width, Height, MM_Status;` |
| Get Notify message | `MM_Get_Notify Msg, MM_Status;` |
| Calibration | `MM_Calib Move_Type, Pos_Jps, Wait_time \Ext;` |
| Stop Mech-Viz project | `MM_Stop_Viz MM_Status;` |
| Get system status | `MM_Get_Status MM_Status;` |

`MM_Lite_Vis` / `MM_Lite_Viz` `Recv_Data_Type`: 1 = vision points w/o custom data (then `MM_Get_Pose`); 2 = vision points with custom data (then `MM_Get_DyData`); 3 = waypoints as joint positions (then `MM_Get_Jps`); 4 = waypoints as TCP (then `MM_Get_Pose`).

**Canonical RAPID flow** (`MM_S1_Vis_Basic`, verbatim core):
```rapid
MODULE MM_S1_Vis_Basic
LOCAL VAR num pose_num:=0; LOCAL VAR num status:=0;
LOCAL VAR num label:=0;    LOCAL VAR num toolid:=0;
LOCAL CONST jointtarget home:=[[0,0,0,0,90,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
LOCAL CONST jointtarget snap_jps:=[[0,0,0,0,90,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
LOCAL PERS robtarget pickpoint:=[[500,100,300],[0.00226227,-0.99991,-0.00439596,0.0124994],[0,0,0,0],[9E+9,...]];
PROC Sample_1()
    AccSet 50, 50;  VelSet 50, 1000;
    MoveAbsJ home\NoEOffs,v3000,fine,gripper1;
    MM_Init_Socket "127.0.0.1",50000,300;          ! once only
    MoveL camera_capture,v1000,fine,gripper1;
    MM_Open_Socket status;
    IF status = 3099 THEN TPWrite "MM: Communication Error"; STOP; ENDIF
    MM_Start_Vis 1,0,2,snap_jps,status;
    IF status<>1102 THEN TPWrite "MM: Status Error"; STOP; ENDIF
    MM_Get_VisData 1,pose_num,status;
    IF status<>1100 THEN Stop; ENDIF                ! 1003 no cloud in ROI / 1002 no result
    MM_Close_Socket;
    MM_Get_Pose 1,pickpoint,label,toolid;
    MoveJ pick_waypoint,v1000,z50,gripper1;
    MoveL RelTool(pickpoint,0,0,-100),v1000,fine,gripper1;
    MoveL pickpoint,v300,fine,gripper1;
    ...
ENDPROC
ENDMODULE
```
Note the pattern: **open socket → commands → close socket → then store poses**. The docs warn: *"If the communication is not closed for a long time, an error that indicates that communication was abnormally closed may be returned for robot programs."*

### 4.3 KUKA (KRL)

Source: [KUKA Standard Interface Commands](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/kuka-interface-commands.html) · [Setup](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/kuka-setup-instructions.html)

**Prerequisites:** 6-axis; **KR C4 or KR C5**; **KSS 8.2 / 8.3 / 8.5 / 8.6 / 8.7**; add-on **Ethernet KRL** with a version pinned to KSS:

| KSS | Ethernet KRL |
|---|---|
| 8.2 or 8.3 | 2.2.8 |
| 8.5 | 3.0.3 |
| 8.6 | 3.1.2.29 |
| 8.7 | 3.2.2.16 |

Ethernet port: **X66** (KR C4 Compact), **KLI** (other KR C4), **XF5** (KR C5). Expert mode required (default password `kuka`).

**Files in the `KUKA` folder and their destinations (verbatim):**
- `mm_module.src`, `mm_module.dat` — program files → **`KRC:\R1\mm`** (create `mm` if absent)
- `MM_COMTEST.src`, `MM_COMTEST.dat` — communication test → **`KRC:\R1\mm`**
- `XML_Kuka_MMIND.xml` — network configuration file → **`C:\KRC\ROBOTER\Config\User\Common\EthernetKRL`**
- `sample/` — example programs

`XML_Kuka_MMIND.xml` (verbatim):
```xml
<ETHERNETKRL>
  <CONFIGURATION>
    <EXTERNAL><IP>192.168.1.121</IP><PORT>50000</PORT></EXTERNAL>
    <INTERNAL><ALIVE Set_Flag="873"/></INTERNAL>
  </CONFIGURATION>
  <RECEIVE><RAW><ELEMENT Tag="MMRecv" Type="BYTE" Set_Flag="871" Size="660" /></RAW></RECEIVE>
  <SEND><RAW><ELEMENT Tag="MMSend" Type="BYTE" Size="660"/></RAW></SEND>
</ETHERNETKRL>
```
`<IP>` = IPC address, `<PORT>` = host port from Robot Communication Configuration. Flag **873** = connected to vision system (ALIVE); flag **871** = data received (RECEIVE). Payload buffer **660 bytes** each way.

| Feature | Routine (exact signature) |
|---|---|
| Communication init | `MM_Init_Socket(XML_Name, Alive_Flag, Recv_Flag, Time_Out)` — e.g. `MM_Init_Socket("XML_Kuka_MMIND",873,871,60)`; `Time_Out` in **seconds**; `XML_Name` case-sensitive, no extension |
| Run Mech-Vision project | `MM_Start_Vis(Job, Pos_Num_Need, SendPos_Type, MM_J, MM_Status)` |
| Get vision result | `MM_Get_VisData(Job, Pos_Num, MM_Status)` |
| Store result/path (TCP) | `MM_Get_Pose(Serial, MM_P, MM_Label, MM_ToolId)` |
| Store path (joint positions) | `MM_Get_Jps(Serial, MM_J, MM_Label, MM_ToolId)` |
| Switch parameter recipe | `MM_Switch_Model(Job, Model_Number, MM_Status)` |
| Get planned path (Mech-Vision) | `MM_Get_Vispath(Job, Jps_Pos, Pos_Num, VisPos_Num, MM_Status)` |
| Get Mech-Vision custom data | `MM_Get_Dy_Data(Job, Pos_Num, MM_Status)` |
| Store Mech-Vision custom data | `MM_Get_DyPose(Serial, MM_P, MM_Label, MM_UserData)` |
| Get gripper DO list | `MM_Get_DoList(Resource, BlockNum, MM_Status)` |
| Set gripper DO list | `MM_Set_DoList(LoopIndex)` |
| Run Mech-Viz project | `MM_Start_Viz(SendPos_Type, MM_J, MM_Status)` |
| Set branch exit port | `MM_Set_Branch(Branch_Num, Export_Num, MM_Status)` |
| Set current index | `MM_Set_Index(Skill_Num, Index_Num, MM_Status)` |
| Get planned path (Mech-Viz) | `MM_Get_VizData(Jps_Pos, Pos_Num, VisPos_Num, MM_Status)` |
| Get Vision Move / custom data | `MM_Get_PlanData(Resource, Jps_Pos, Pos_Num, VisPos_Num, MM_Status)` |
| Store Vision Move / custom data | `MM_Get_PlanPose(Serial:IN, Jps_Pos:IN, MM_P:OUT, MM_MoveType:OUT, MM_ToolNum:OUT, MM_Speed:OUT)` *(when `Jps_Pos` is 2 or 4)* |
| Read Mech-Viz Step parameter | `MM_Get_Property(Get_Id, MM_Status, Viz_Prop)` |
| Set Mech-Viz Step parameter | `MM_Set_Property(Set_id, MM_Status)` |
| Input object dimensions | `MM_Set_BoxSize(Job, Length, Width, Height, MM_Status)` |
| Get Notify message | `MM_Get_Notify(MM_Notify, MM_Status)` |
| Calibration | `MM_Calib(Move_Type, PosJps, WaitTime, E1)` |
| Stop Mech-Viz project | `MM_Stop_Viz(MM_Status)` |
| Get system status | `MM_Get_Status(MM_Status)` |
| **Vision pose → BASE frame** | `MM_Get_Wobj(Serial, MM_Frame_num, MM_Label, MM_ToolId)` — **KUKA-only**; stores the pose into a **base coordinate variable** (e.g. `MM_Get_Wobj(1,10,label,toolid)` → base variable 10) |

### 4.4 YASKAWA (INFORM + MotoPlus)

Source: [YASKAWA Standard Interface Commands](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/yaskawa-interface-commands.html) · [Setup](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/yaskawa-setup-instructions.html)

**Prerequisites:** 6-axis YASKAWA. Controller / system software: **DX200 → DN3.16.00A-00**, **YRC1000 → YAS2.94.00-00**, **YRC1000micro → YBS2.31.00-00**. **`ETHERNET` option must be `USED`** and **MotoPlus `APPLI. AUTOSTART AT POWER ON` must be `ENABLE`**. Ethernet: YRC1000 → **LAN2 (CN106)** on the CPU board (LAN1 is teach-pendant only; use LAN3 if LAN2 occupied); DX200 → **CN104**.

**Files (verbatim):**
- **`mm_module_yrc1000.out`** — background program (MotoPlus application) for YRC1000
- **`mm_module_dx200.out`** — background program for DX200
- **`JBI/`** — folder holding the **foreground program files** (the `MM_*` jobs)
- **`sample/`** — example programs

Load path: MotoPlus `.out` via maintenance mode → `MotoPlus APL. → DEVICE USB: Pendant → LOAD(USER APPLICATION)`; verify via `MotoPlus APL. → FILE LIST`. Jobs via `EX. MEMORY FOLDER → JBI → EX. MEMORY LOAD → JOB → EDIT → SELECT ALL`. **Only one `.out` may be loaded** on most controllers — an existing one must be deleted first, or `FATAL:MM:SOCKET_OPEN_ERROR` results. Test job: **`MM_COMTEST`** (edit line 0001 IP/port; default port 50000).

Arguments are passed as a **single semicolon-delimited string** via `ARGF`.

| Feature | Job call (exact signature) |
|---|---|
| Initialize communication | `MM_INIT_SOCKET("IP_Address;Server_Port;Time_Out")` — `Time_Out` in **minutes** |
| Establish TCP communication | `MM_OPEN_SOCKET` |
| Close TCP communication | `MM_CLOSE_SOCKET` |
| Run Mech-Vision project | `MM_START_VIS("job;pos_num_need;sendpos_type;prNum;regStatus")` |
| Get vision result | `MM_GET_VISDATA("Job;Pose_Num;MM_Status")` |
| Store result/path (TCP) | `MM_GET_POSE("Index;Robtarget;Label;Tool_Id")` — P variable type must be **robot** |
| Store path (joint positions) | `MM_GET_JPS("Index;Jointtarget;Label;Tool_Id")` — P variable type must be **joint position or pulse** |
| Switch parameter recipe | `MM_SET_MODEL("Job;Model_number;regStatus")` |
| Get planned path (Mech-Vision) | `MM_GET_VISPATH("job;GetPos_Type;Pos_Num;VisPos_Num;regStatus")` |
| Get Mech-Vision custom data | `MM_GET_DYDATA("job;regPosNum;regStatus")` |
| Store Mech-Vision custom data | `MM_GET_DYPOSE("serial;prNum;regLabel;rrNum")` |
| Get gripper DO list | `MM_GET_DOLIST("Resource;blockNum;regStatus")` |
| Set gripper DO list | `MM_SET_DOLIST("loop_index")` |
| Run Mech-Viz project | `MM_START_VIZ("sendpos_type;prNum;regStatus")` |
| Set branch exit port | `MM_SET_BRANCH("Branch_Num;Exit_Num;regStatus")` |
| Set current index | `MM_SET_INDEX("Skill_Num;Index_Num;regStatus")` |
| Get planned path (Mech-Viz) | `MM_GET_VIZDATA("GetPos_Type;Pos_Num;VisPos_Num;regStatus")` |
| Get Vision Move / custom data | `MM_GET_PLANDATA("Resource;jpsPos;regPosNum;visPosNum;regStatus")` |
| Store Vision Move / custom data | `MM_GET_PLANPOSE("serial;prNum;brNum;rrNum")` |
| Read Mech-Viz Step parameter | `MM_GET_PROPERTY("Read_id;regStatus;Viz_Prop")` |
| Set Mech-Viz Step parameter | `MM_SET_PROPERTY("Write_id;regStatus")` |
| Input object dimensions | `MM_SET_BOXSIZE("Job;Length;Width;Height;regStatus")` |
| Get Notify message | `MM_GET_NOTIFY("NotifyMsg")` |
| Calibration | `MM_CALIB("Move_Type;Pos_Jps;Wait_Time;Rnum;Ext;Pos")` |
| Stop Mech-Viz project | `MM_Stop_Viz("regStatus")` |
| Get system status | `MM_GET_STATUS("regStatus")` |

**Canonical INFORM flow** (`MM_S1_Vis_Basic`, verbatim core):
```
NOP
CLEAR I050 20
SUB P070 P070 / SUB P071 P071
SETE P070 (3) 100000                                  ' 100 mm Z offset (micron units)
MOVJ C00000 VJ=50.00
CALL JOB:MM_INIT_SOCKET ARGF"192.168.170.22;50000;1"  ' once only
MOVJ C00001 VJ=50.00 PL=0                             ' image-capture pose
CALL JOB:MM_OPEN_SOCKET
CALL JOB:MM_START_VIS ARGF"1;0;2;30;80"
IFTHENEXP I080<>1102 / PAUSE / ENDIF
CALL JOB:MM_GET_VISDATA ARGF"1;51;52"
IFTHENEXP I052<>1100 / PAUSE / ENDIF                  ' 1003 no cloud in ROI, 1002 no result
CALL JOB:MM_CLOSE_SOCKET
CALL JOB:MM_GET_POSE ARGF"1;71;61;62"
MOVJ C00002 VJ=50.00
SFTON P070 / MOVL P071 V=166.6 PL=0 / SFTOF           ' approach via shift
MOVL P071 V=50.0 PL=0                                 ' pick
...
END
```

### 4.5 Robot-side error strings (not status codes)

Source: [FANUC Error Messages](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/fanuc-error-messages.html) · [YASKAWA Error Messages](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/yaskawa-error-messages.html) · [ABB](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/abb-error-messages.html) · [KUKA](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/kuka-error-messages.html)

| Brand | Strings |
|---|---|
| FANUC | `MM:Robot_Internal_Error [X]`, `MM:Robot_Socket_Closed`, `MM:Robot_Argument_Error [X]`, `MM:Robot_CMD_Error` |
| YASKAWA | `FATAL:MM:INTERNAL_ERROR:ERROR_ID`, `FATAL:MM:SOCKET_OPEN_ERROR`, `SOCKET_CONNECT_ERROR`, `SOCKET_SEND_ERROR`, `SOCKET_SELECT_ERROR`, `SOCKET_RECV_ERROR`, `SOCKET_RECV_TIMEOUT`, `SOCKET_CLOSED`, `FATAL:MM:ARGUMENT_MISSING`, `FATAL:MM:ARGUMENT_INVALID` |

`MM:Robot_CMD_Error` means either the command code sent by the foreground program was not found, or the code received by the background program does not match what the vision system sent — i.e. a **calling-sequence violation**.

---

## 5. Networking and configuration

### 5.1 Robot Communication Configuration

Opened from the Mech-Vision toolbar (2.2.1 path: *Robot and Communication → Robot Communication Configuration*). Required settings for Standard Interface over TCP:

1. **Select robot** → *Listed robot* → **Select robot model** → Next.
2. **Interface service type** = **Standard Interface**.
3. **Protocol** = **TCP Server**.
4. **Protocol format** — brand-dependent (see 5.3).
5. **Port number** — must not be occupied.
6. *Robot integration → **Open program folder*** — source of the files to load.
7. Optional: **Auto enable interface service when opening the solution**.
8. **Apply**, then verify the **Robot Communication Configuration toggle on the toolbar is flipped and blue** — the interface service must be running or nothing responds.

**Advanced Settings** (*Robot Communication Configuration → Next → Advanced Settings*) holds: **maximum number of poses to obtain each time** (default 20, limit 30), **Timeout for getting Mech-Vision data(s)** (default 10 s), and **Property Configuration** (opens `property_config` for commands 207/208).

### 5.2 IP addresses and ports

Source: [How to Configure IP Address and Port Number in Robot Communication Configuration?](https://docs.mech-mind.net/en/robot-integration/latest/faq/faq-13.html)

| Protocol | Vision system role | IP address field | Port field |
|---|---|---|---|
| **TCP Server** | Server | **`0.0.0.0`** | Custom. *"The Standard Interface program on the external device requires the IP address of the IPC in which the vision system is installed and the port number set here."* |
| Siemens PLC Client | Client | IP of the PLC | none |
| ETHERNET IP | Server | none | none |
| MODBUS TCP Slave | Server | `0.0.0.0` | custom |
| UDP Server | (no role) | `0.0.0.0` | custom |
| Mitsubishi MC Client | Client | IP of the PLC | Port of the PLC |

**On the port question:** there is **no single hard-coded default**. Guidance and observed values:
- ABB / KUKA / YASKAWA setup pages, verbatim: *"It is recommended to set the port number to **50000 or above**. Ensure that the port number is not occupied by another program."*
- Official example programs use **50000** (ABB, KUKA XML, YASKAWA) and **30000** (FANUC `MM_S1_Vis_Basic`; FANUC `MM_INIT_SKT` doc example also 30000).
- FANUC constraint: *"If the robot system version is V10, the value range for `Svr_Port` is 1 to 32767."* — so **50000 is invalid on FANUC V10**; use ≤32767 there.
- The FANUC setup page does not repeat the "50000 or above" recommendation; it only says the port must be free.

IPC and robot controller **must be in the same subnet** with different addresses (docs example: `192.168.100.169/255.255.255.0` and `192.168.100.170/255.255.255.0`). On FANUC, if Port 1 (CD38A) and Port 2 (CD38B) are both enabled, *"their IP addresses must be on different subnets."*

**Multi-port / multi-workstation:** *"Communication over TCP protocol supports concurrent processing of project data with a maximum of **four ports**… If multiple ports on the IPC are connected with the robot side, you should modify the example program to ensure that the global variables are not shared by different ports."* Snap7 likewise supports max 4 DBs. FANUC exposes 8 client tags (`C_Tag` 1–8). ([Standard Interface Development Manual](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-development-manual/standard-interface-development-manual.html))

### 5.3 Protocol format per robot brand (Standard Interface, TCP Server)

| Brand | Protocol format | Source |
|---|---|---|
| **FANUC** V7 / V8 / V9 | **HEX (big-endian)** | FANUC setup |
| **FANUC** V10 | **HEX (little-endian)** | FANUC setup |
| **ABB** (RobotWare 6) | **HEX (little-endian)** | ABB setup |
| **KUKA** | **HEX (little-endian)** | KUKA setup |
| **YASKAWA** | **ASCII** | YASKAWA setup |

### 5.4 Supported Standard Interface transports

Source: [Standard Interface Development Manual](https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-development-manual/standard-interface-development-manual.html)

| Protocol | Vision system role | Notes |
|---|---|---|
| TCP | Server | multiple ports (max 4 concurrent) |
| UDP | Server | *"Not yet available"* (no command guide published) |
| Siemens PLC Snap7 | **Client** | multiple DBs (max 4) |
| PROFINET | Slave device | |
| EtherNet/IP | Slave device | |
| Modbus TCP | Slave device | |
| Mitsubishi MC | **Client** | |

### 5.5 Standard Interface vs Master-Control vs Adapter

Source: [Communication Modes](https://docs.mech-mind.net/en/robot-integration/latest/communication-basics/communication-modes.html)

| | **Master-Control** | **Standard Interface** | **Adapter** |
|---|---|---|---|
| Command type | Robot commands | Standard interface commands | Custom commands |
| Command **sender** | **Vision system** | External device (robot / PLC / host) | External device |
| Command **receiver** | **Robot** | Vision system | Vision system |
| Who writes code | *"No programs are needed"* | Programs written **for the external device** | Programs for **both** vision system and external device |
| Protocols | TCP, UDP | TCP, UDP, Siemens Snap7, PROFINET, EtherNet/IP, Modbus TCP, Mitsubishi MC | all of the above **+ HTTP, WebSocket, others** |
| Difficulty / flexibility | Low / Low | Medium / Medium | High / High |
| Gluing supported | ✓ | **✗** | ✓ |
| Workpiece loading, palletizing/depalletizing, locating & assembly, piece picking | ✓ | ✓ | ✓ |

Key verbatim distinctions:
- *"In the standard interface communication, the vision system only sends data in response to commands from the external device and **does not control** the external device."*
- Master-Control: *"the vision system acts as the controller and the robot acts as the controlled party."* Two implementations: **loading-required** (ABB, FANUC, KUKA — master-control programs must be loaded and running) and **loading-free** (AE, JAKA — robot switched to remote control mode, vision system drives it via robot SDKs).
- *"Standard Interface Communication can be understood as a custom version of Adapter Communication. Due to their resemblance, Standard Interface Communication and Adapter Communication are collectively referred to as **interface communication**."*
- Standard Interface commands are *"Developed by Mech-Mind based on the standard communication protocol"* and *"define request and response formats"* — i.e. the protocol is fixed by Mech-Mind. Adapter lets you define protocol, request and response formats yourself (Python; via *Adapter Generator* or *Adapter Programming Guide*).

**For machine tending specifically:** Standard Interface covers workpiece loading, so it is the right mode; note the documented gap is **gluing only**.

---

## 6. Notify / "empty tray" signalling

There is **no dedicated "empty tray" command** anywhere in the Standard Interface command set. The mechanism for the vision/planning side to tell the robot "condition X occurred" is the **Notify Step** surfaced by **Command 601**:

- Mech-Vision side: connect **Notify** to the right of another Step (e.g. **Output**); in the Output Step's parameters select **Trigger Control Flow Given Output**; in the Notify Step set **Service Name = `Standard Interface Notify`** (a *required* literal value) and **Message** = a positive integer (e.g. `1001`).
- Mech-Viz side: place **Notify** in the workflow, select **Standard Interface** as the receiver, set **Message** to a positive integer (e.g. `1000`).
- Reply: `601, <message>` — **no status code**, just the integer.
- **Timing hazard (verbatim):** *"When the Notify Step is executed in the Mech-Vision or Mech-Viz project, the message remains in the buffer of the vision system for only **three seconds**. Therefore, you should consider the timing of calling this command to ensure successful message retrieval."* Docs require calling 601 *"immediately AFTER Command 101 or Command 201."*
- Range **6001~6199** is documented as *"Status codes that can be customized in Mech-Vision"* — this is the sanctioned block for user-defined application states such as "tray empty". **UNVERIFIED:** I found the range declaration but no page describing *how* to author/emit 6001–6199 codes.
- Practical alternatives for "tray empty" that are fully documented: status **1002** *No vision result* / **1003** *No point cloud in ROI* from 102, or **2039 / 1044** *No vision point for the Vision Move Step*.

---

## 7. Explicitly UNVERIFIED / not found

1. **`CV-E0000`** — no page enumerating `CV-Exxxx` codes; only the generic statement that `CV-Exxxx` is a Mech-Vision project runtime error surfaced as status **1015**.
2. **`exit_code`** — no such field in any Standard Interface command, reply, or status-code page.
3. **5xxx status-code range** — does not exist in the Standard Interface code space (ranges are 1xxx, 2xxx, 3xxx, 4xxx, 6xxx, 7xxx only).
4. **`3099`** — used in every official example program as "failed to open socket" but **absent from the official status-code table**.
5. **1104** (solution switched) and **1111** (global variable set/get) — used in the Command 104 / 504–507 pages but **missing from the 2.2.1 status-code table**.
6. **FANUC installed-file manifest** — the `INSTALL` procedure is documented but the individual KAREL `.pc` / TP `.tp` / variable `.vr` file names are not published. Confirmed program names only: `MM_COMTEST`, and the `MM_S1`–`MM_S22` samples.
7. **YASKAWA `JBI` job manifest** — the folder is documented and each `MM_*` job name is documented as a callable, but there is no explicit file listing.
8. **Command 502 in 2.x** — present in 1.7.4 (Input TCP to Mech-Viz / External Move Step, success 2107) but absent from the 2.2.1 TCP command list, even though status code **2107** ("Set move point for External Move Step successfully") survives in the 2.2.1 code table. Whether 502 still works in 2.2.1 is unconfirmed.
9. **UDP Standard Interface command reference** — officially *"Not yet available."*
10. **"empty tray" command** — does not exist; use Notify (601) or the 6001–6199 custom range.
11. **`sendPose`** — not a Mech-Mind identifier. The real parameter is `SendPos_Type` / `sendpos_type`.

---

## Source URLs

- TCP/IP Interface Commands (2.2.1): https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-development-manual/tcp-ip-socket.html
- TCP/IP Interface Commands (1.7.4): https://docs.mech-mind.net/en/robot-integration/1.7.4/standard-interface-development-manual/tcp-ip-socket.html
- Status Codes and Troubleshooting (2.2.1): https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-development-manual/status-codes-error-troubleshooting.html
- Status Codes and Troubleshooting (1.7): https://docs.mech-mind.net/1.7/en-GB/SoftwareSuite/MechCenter/MechInterface/StandardInterfaceDevelopmentManual/StatusCodesAndErrorTroubleshooting/StatusCodesAndErrorTroubleshooting.html
- Status Codes and Trouble Shooting (1.6): https://docs.mech-mind.net/1.6/en-GB/SoftwareSuite/MechCenter/MechInterface/StandardInterfaceDevelopmentManual/StatusCodesAndErrorTroubleshooting/StatusCodesAndErrorTroubleshooting.html
- Calling Sequence of Standard Interface Commands: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-development-manual/control-timing.html
- Standard Interface Development Manual (protocol matrix): https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-development-manual/standard-interface-development-manual.html
- Communication Modes: https://docs.mech-mind.net/en/robot-integration/latest/communication-basics/communication-modes.html
- Robot Communication Configuration: https://docs.mech-mind.net/en/suite-tutorial/latest/vision-system-communication-configuration.html
- FAQ 13 — IP address and port: https://docs.mech-mind.net/en/robot-integration/latest/faq/faq-13.html
- Error Code Processing: https://docs.mech-mind.net/en/suite-service-manual/latest/troubleshooting/troubshooting-common-issues-error-codes.html
- FANUC commands: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/fanuc-interface-commands.html
- FANUC setup: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/fanuc-setup-instructions.html
- FANUC example programs: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/fanuc-example-program.html
- FANUC MM_S1_Vis_Basic: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/fanuc-s1-vis-basic.html
- FANUC error messages: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/fanuc-error-messages.html
- ABB commands: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/abb-interface-commands.html
- ABB setup (RobotWare 6): https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/abb-setup-instructions.html
- ABB MM_S1_Vis_Basic: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/abb-s1-vis-basic.html
- ABB error messages: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/abb-error-messages.html
- KUKA commands: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/kuka-interface-commands.html
- KUKA setup: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/kuka-setup-instructions.html
- KUKA error messages: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/kuka-error-messages.html
- YASKAWA commands: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/yaskawa-interface-commands.html
- YASKAWA setup: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/yaskawa-setup-instructions.html
- YASKAWA MM_S1_Vis_Basic: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/yaskawa-s1-vis-basic.html
- YASKAWA error messages: https://docs.mech-mind.net/en/robot-integration/latest/standard-interface-robot/yaskawa-error-messages.html