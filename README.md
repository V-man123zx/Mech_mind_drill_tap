# AMTE 400 Drill & Tap Machine-Tending Cell
## As-Built Engineering Documentation

**Owner:** ICR247 / GennFlex
**Author / Integrator:** Vedansh (vedansh@icr247.com)
**Document date:** 2026-08-12
**Configuration documented:** Solution backup `MechMaster_drill_Tap_Backup_7_30_2026.zip`, solution timestamp **2026-07-30 18:49:49**
**Status:** Production — single part number running; expansion planned

---

## Table of Contents

1. [Purpose and Scope](#1-purpose-and-scope)
2. [System Overview](#2-system-overview)
3. [Hardware Inventory](#3-hardware-inventory)
4. [Network Architecture](#4-network-architecture)
5. [End-of-Arm Tooling](#5-end-of-arm-tooling)
6. [Hand-Eye Calibration](#6-hand-eye-calibration)
7. [Camera Configuration](#7-camera-configuration)
8. [Target Objects (Part Models)](#8-target-objects-part-models)
9. [Mech-Vision Project: `Drill_Tap`](#9-mech-vision-project-drill_tap)
10. [Path Planning Project](#10-path-planning-project)
11. [Robot Communication — Standard Interface](#11-robot-communication--standard-interface)
12. [Design Decisions and Rationale](#12-design-decisions-and-rationale)
13. [Operating Procedure](#13-operating-procedure)
14. [Performance Data](#14-performance-data)
15. [Known Issues and Troubleshooting](#15-known-issues-and-troubleshooting)
16. [Maintenance](#16-maintenance)
17. [Roadmap](#17-roadmap)
18. [File and Directory Reference](#18-file-and-directory-reference)
19. [Open Items](#19-open-items)
20. [Revision History](#20-revision-history)

---

## 1. Purpose and Scope

This cell automates loading of sheet-metal components into a CNC drill-and-tap machine for the production of the **AMTE 400 table**, sold by GennFlex and ICR247. The AMTE 400 requires several drilled and tapped components; this cell currently handles **one large component** (`t1`), with additional smaller parts planned.

The cell replaces manual loading of a randomly stacked bin of laser-cut steel bars. A Yaskawa HC10 collaborative robot, guided by an overhead Mech-Eye NANO ULTRA 3D camera, identifies an individual bar in the bin, picks it with a custom three-electromagnet end effector, and presents it to the CNC fixture.

Bin logistics are handled by a **Pudu mobile robot (AMR)**, which delivers a full pick bin to the camera station and removes it when empty — extending automation beyond the cell itself into shop-wide material flow.

**In scope:** vision system, robot motion planning, calibration, communication, operating and recovery procedures.
**Out of scope:** CNC machine programming, Pudu fleet management, table assembly downstream of drilling/tapping.

---

## 2. System Overview

### 2.1 Process Flow

```
Pudu AMR delivers pick bin
          ↓
Bin positioned under overhead camera
          ↓
Operator starts robot job
          ↓
   ┌──► Robot sends cmd 101 → Mech-Vision runs Drill_Tap
   │              ↓
   │    Camera captures 2D + 3D → 3D matching against t1 CAD model
   │              ↓
   │    Pose filtering, sorting, single-pick selection
   │              ↓
   │    Path Planning generates JOINT-SPACE trajectory
   │              ↓
   │    Robot sends cmd 105 → receives J1–J6 waypoints
   │              ↓
   │    Robot enters bin, energizes magnets, picks, retreats
   │              ↓
   │    Robot loads part into CNC fixture → drill/tap cycle
   │              ↓
   └──── Loop until no parts detected → robot pauses
          ↓
Pudu AMR removes empty bin
```

### 2.2 Software Stack

| Component | Version | Notes |
|---|---|---|
| Mech-Vision | 2.1.1 (UI) — project created 2.1.1, last modified **2.1.2** | Solution `MechMaster`, project `Drill_Tap` |
| Path Planning project | 2.1.2 | `planning_project`, lite mode |
| Communication Component | Bundled with Mech-Vision solution | Stock Mech-Mind build, unmodified |
| Mech-Eye SDK | *TBD* | |
| Camera firmware | *TBD* | |
| Robot controller | YRC1000micro | |
| IPC OS | Windows 10 IoT Enterprise LTSC | Factory image on Mech-Mind IPC STD |

> **Note on Mech-Viz:** The solution contains a folder `Mech-Viz-YMfUNP/` and the solution file references `viz_project: planning_project`. There is **no separately authored Mech-Viz workcell program**. All motion planning is performed by the **Path Planning step inside Mech-Vision**, which uses the lightweight `planning_project` as its planning container. See [§12.1](#121-joint-space-output-instead-of-cartesian) for why.

---

## 3. Hardware Inventory

### 3.1 Robot

| Item | Value |
|---|---|
| Model | Yaskawa **HC10** collaborative robot |
| Controller | Yaskawa **YRC1000micro** |
| Configured type in solution | `YASKAWA_HC10` |
| Base frame | Coincident with planning world origin (pose = identity) |
| Mounting | Floor / pedestal-mounted on blue fabricated frame with casters |
| Soft joint limits (as configured in planning project) | J1 ±180°, J2 ±180°, **J3 −5° to +355°**, J4 ±180°, J5 ±180°, J6 ±180° |

> The asymmetric J3 limit (−5° to +355°) is deliberate and important — the pick poses operate around **J3 ≈ +234° to +266°**, well outside a conventional ±180° range. Do not "normalize" these limits.

### 3.2 3D Camera

| Item | Value |
|---|---|
| Model | Mech-Eye **NANO ULTRA** (structured light) |
| Serial number | `RUM70249B500E015` |
| IP address | `192.168.5.10` |
| Mounting | **Eye-to-hand**, fixed overhead on aluminium extrusion above the pick bin |
| Position in robot base frame | X = −60.05 mm, Y = +992.11 mm, Z = +625.25 mm |
| Orientation in robot base frame (XYZ static Rx,Ry,Rz) | −179.479°, −0.437°, −91.975° |
| Variant | **NANO ULTRA-700M** (700 mm object focal distance) — inferred from the ~761 mm working distance in use |
| Power | 24 VDC, 3.75 A (idle 8 W, average 12 W, peak 37 W) |
| Working depth range configured | 200 – 1000 mm |
| 2D image resolution | 2400 × 1800 |
| Manufacturer accuracy spec | 0.1 mm @ 0.6 m (VDI/VDE 2634 Pt 2); Z repeatability 0.1 mm @ 0.6 m |
| Typical capture time | 0.5 – 0.9 s |
| Recommended warm-up | **30 minutes** |

#### 3.2.1 ⚠ Field of View Is the Binding Constraint

The camera sits **625.25 mm** above the robot base and the bin ROI is centred at **Z = −136 mm**, giving a working distance of approximately **761 mm** — near the far end of the NANO ULTRA-700M's 400–800 mm range.

At that distance the manufacturer's field of view is roughly **740 × 525 mm** (interpolated between the published 400 × 270 mm @ 0.4 m and 770 × 550 mm @ 0.8 m).

This has two consequences that are easy to miss:

1. **The configured `3d_roi` (1011.9 × 672.9 mm) is larger than the camera can actually see.** The ROI is not the limiting aperture — the FOV is. Widening the ROI will not reveal more of the bin.
2. **Part `t1` is 727 mm long against a ~740 mm FOV width.** The bar only just fits across the frame. A bar lying diagonally, or a bin positioned even slightly off-centre, can push one end of a part outside the FOV — producing a partial point cloud and a failed or low-confidence match.

This is a plausible contributing factor to the rare mis-pick described in [§15.2](#152-false-pick-pose-with-12-parts-remaining), and it is a **hard constraint on any future longer part**. Anything appreciably longer than `t1` will require either raising the camera (which pushes past the 800 mm spec limit and degrades accuracy) or a different camera variant.

### 3.3 Industrial PC

| Item | Value |
|---|---|
| Model | Mech-Mind **IPC STD** — `IPCW-i5-16G-512G` |
| CPU / RAM / Storage | Intel Core i5-12400 / 16 GB DDR4 / 512 GB SSD |
| Network ports | 4 × 2.5 GbE, factory-labelled **CAMERA 1**, **CAMERA 2**, **CAMERA 3 / ROBOT**, **ROBOT / PLC** |
| Power input | DC 12–28 V, dual input (Power 1 / Power 2); **Power 1 in use** |
| Ports in use | 1 × camera Ethernet, 1 × robot Ethernet, USB 3.0 |

> The IPC chassis carries a hot-surface warning label. Allow it to cool before servicing.

### 3.4 Other Cell Equipment

| Item | Detail |
|---|---|
| CNC drill/tap machine | *TBD — make/model* |
| Pick bin | Fabricated black steel bin on aluminium-extrusion stand, positioned under the camera |
| Mobile robot | Pudu AMR — delivers and removes the pick bin |
| Guarding | Blue mesh fence panels with clear curtain sections |
| Operator station | HMI pendant with e-stop, plus Yaskawa teach pendant |

---

## 4. Network Architecture

Two isolated subnets are used, separating high-bandwidth camera traffic from robot control traffic.

| Device | Subnet | IP | Role |
|---|---|---|---|
| Mech-Eye NANO ULTRA | 192.168.5.x | `192.168.5.10` | Camera — connects to IPC **CAMERA** port |
| IPC (camera NIC) | 192.168.5.x | *TBD* | |
| IPC (robot NIC) | 192.168.2.x | `192.168.2.1` (observed) | Hosts Standard Interface TCP server |
| Yaskawa YRC1000micro | 192.168.2.x | `192.168.2.31` | TCP **client** to the IPC |

**Standard Interface listener:** `0.0.0.0:50000` (binds all interfaces).

### 4.1 Recommendations

- Enable **jumbo frames** on the camera NIC if not already set (see Mech-Eye manual §9.7) to reduce capture latency.
- The camera was observed dropping and re-establishing its connection once in the 2026-07-30 log (`Camera 192.168.5.10 get disconnected` → reconnected 1 s later). Isolated, but worth monitoring — see [§15.4](#154-camera-disconnect).

---

## 5. End-of-Arm Tooling

The EOAT is a **custom design by the integrator** — three electromagnets mounted on compliant springs.

| Item | Value |
|---|---|
| Type | 3 × electromagnet, spring-mounted |
| Purpose | Pick long, flat, ferrous sheet-metal bars from a random stack |
| Compliance | Springs allow each magnet to conform to a bar that is tilted or resting on others |
| TCP translation | X = 0, Y = 0, **Z = 166 mm** from flange |
| TCP rotation | **Rz = −65.0°** (Rx = Ry = 0) |
| Tool name in tool library | `Default tool` (GUID `{608ecf20-…acb41}`) |
| Collision model | **None loaded** (`ee_collision_model_name` empty, bbox 0,0,0) |
| Pick point symmetry | 360° |
| Magnet voltage / DO mapping | *TBD* |
| Part-present feedback | *TBD — believed none* |

### 5.1 Design Notes

The spring mounting is what makes single-magnet-per-contact picking work on a stack of nominally flat but slightly warped 7.94 mm (5/16") bars. Rigid magnets would only make contact on one of the three faces and would either fail to lift or lift with an unpredictable tilt.

### 5.2 ⚠ Open Risk — No Tool Collision Model

The end effector has **no collision geometry defined** in the tool library, and point-cloud collision checking is **disabled** in the planning executor ([§10.2](#102-executor-settings)). Collision avoidance therefore relies on:

- the bin-entry and bin-exit fixed joint waypoints being safe by construction,
- the approach/retreat relative moves being short and vertical,
- the parts lying flat in a known bin.

This is workable for the current single-part, well-constrained application, but it should be documented as a **known limitation** before the cell is expanded to multiple part types or a taller stack. Adding a tool collision model is the single highest-value robustness improvement available.

---

## 6. Hand-Eye Calibration

| Item | Value |
|---|---|
| Type | **Eye-to-hand** (camera fixed, not on robot) |
| Method | **Manual** (3-point touch-up) |
| Calibration board | **OCB-20** |
| Board pattern | 11 × 9 circle grid, **20 mm** point spacing |
| Calibration group name | `RUM70249B500E015-EyeToHand-2026-07-17` |
| Date performed | **2026-07-17** (extrinsics written 14:59:23) |
| Camera parameter group used during calibration | `calib` |

### 6.1 Method Used

Rather than purchasing a rigid calibration board, the OCB-20 pattern was **printed true-to-size and taped to the bottom of the Pudu pick bin**. This puts the calibration plane exactly where the parts will actually sit, which is the correct plane to calibrate against.

Three grid points were touched with the robot TCP (grid indices **0, 10, and 88** of the 11 × 9 array — i.e. two corners of one edge and the opposite corner, giving good angular spread):

| Point | Position in robot base frame (m) |
|---|---|
| Grid index 0 | (0.00631, 0.77490, −0.19767) |
| Grid index 10 | (0.01339, 0.96578, −0.19231) |
| Grid index 88 | (−0.14938, 0.77901, −0.19823) |

Resulting fixed board pose in base frame: (7.58, 771.31, −197.76) mm.

### 6.2 Resulting Extrinsics

Camera pose in robot base frame (`depthInBase`, quaternion form):

```
translation (m):  [-0.06004712, 0.99210570, 0.62525277]
quaternion (w,x,y,z): [0.00041811, -0.69481365, 0.71916531, -0.00592291]
```

Equivalent: **X −60.05 mm, Y +992.11 mm, Z +625.25 mm; Rx −179.479°, Ry −0.437°, Rz −91.975°** — i.e. the camera looks almost straight down, rotated ~92° about the base Z axis relative to the robot frame.

### 6.3 Intrinsics

| Parameter | Value |
|---|---|
| fx, fy | 2282.876, 2282.876 |
| cx, cy | 1362.383, 921.402 |
| Distortion coefficients | all zero (pre-rectified by camera) |
| Modified | 2026-07-17T14:34:58 |

### 6.4 Caveats and Re-calibration Guidance

- **A printed paper board is dimensionally sensitive.** Printer scaling, humidity, and tape wrinkles all introduce error. Verify with a caliper across a known number of grid pitches (e.g. 10 spaces should measure exactly 200.0 mm) before trusting any re-calibration.
- **Three points is the minimum.** The manual 3-point method solves the plane but gives no redundancy and no residual error metric. If pick accuracy degrades, re-calibrate with more points or move to the standard automatic multi-pose calibration.
- **Re-calibrate after any of:** camera or extrusion mount disturbed; robot base moved or re-levelled; the bin stand relocated; a controller home/encoder reset.
- The calibration is tied to the bin's *position*, not the bin itself. Because the Pudu delivers the bin each cycle, **bin drop-off repeatability directly limits pick accuracy** — see [§15.2](#152-false-pick-pose-with-1-2-parts-remaining).

---

## 7. Camera Configuration

The active parameter group is **`drill-tap-cell`** (index 8 of 9 stored groups, derived from the `default` template).

| Parameter | Value | Comment |
|---|---|---|
| `DepthRange` | 200 – 1000 mm | Working volume from camera |
| `Scan3DExposureTime` | **25 ms** | Single exposure |
| `Scan3DExposureCount` | 1 | No HDR stacking — keeps cycle fast |
| `Scan3DGain` | 0 | |
| `Scan2DExposureTime` | 30 ms | |
| `Scan2DExposureMode` | 0 — Timed (single fixed exposure) | Options are Timed / Auto / HDR; value 0 corresponds to Timed, consistent with `Scan2DExposureTime` being the active setting |
| `Scan2DGain` | 0 | |
| `ProjectorFringeCodingMode` | **0 = Fast** | Chosen for speed; the `calib` group uses 1 (Accurate) and `Reflective object` uses 3 (Reflective) |
| `ProjectorPowerLevel` | 0 (High) | |
| `FringeContrastThreshold` | 3 | |
| `PointCloudNoiseRemoval` | 1 | |
| `PointCloudOutlierRemoval` | 2 | |
| `PointCloudSurfaceSmoothing` | 3 | Highest smoothing — appropriate for flat sheet-metal faces |
| `PointCloudEdgePreservation` | 1 | |
| `Scan3DBinningEnable` | false | Full resolution |
| `EdgeArtifactRemoval` | false | |

### 7.1 Notes

- **Fast fringe coding with heavy surface smoothing** is a deliberate speed/quality trade: the parts are large, flat, and matte-to-lightly-reflective mill-finish steel, so surface detail matters less than throughput. If new parts are shinier, evaluate the stored `Reflective object` group.
- Other stored groups (`Tejas_Shaft`, `Tejas_Wheel`, `red_wheel_cover`, `test`, `Reflective piece welding`) belong to **other projects on this camera** and are not used by this cell. Do not delete them without checking.

---

## 8. Target Objects (Part Models)

Two AMTE 400 components are modelled in the workobject library. Only `t1` is in the production flow today.

### 8.1 `t1` — Active Production Part

| Item | Value |
|---|---|
| Source CAD | `LSR-M4-8_E Support Copy.stl` |
| Bounding box | **727.05 × 71.40 × 7.94 mm** |
| Material thickness | 7.94 mm (5/16") |
| Import method | ImportCAD, scale factor 0.001 (mm → m) |
| Model GUID | `{2d529e21-32e1-42eb-8a8b-536e95ec7156}` |
| Collision model | `t1.binvox`, voxel resolution 2.0 mm |
| Matching models | `surface.ply` + `edge.ply` (edge model marked stable) |
| Geometric constraint | Enabled |
| Symmetry ranges | [90°, 90°, 180°], steps [180°, 180°, 180°] |
| Pick points | 1 (`PickPoint_1`) |
| Pick point offset | [0, −0.534 mm, +0.031 mm] |
| Pick relaxation — translation | ±20 mm in X/Y/Z, 5 mm steps |
| Pick relaxation — rotation | ±20° about X/Y/Z, 5° steps |

> The generous ±20 mm / ±20° pick relaxation is what lets the planner find a reachable solution on a bar lying at an awkward angle. It works because the three spring-mounted magnets tolerate contact-point variation — a rigid gripper could not use relaxation this wide.

### 8.2 `t2` — Modelled, Not Yet in Production

| Item | Value |
|---|---|
| Source CAD | `blender_modified_MM_Drill_Tap_wing_peice.stl` |
| Bounding box | **735.94 × 87.01 × 7.94 mm** |
| Model GUID | `{8cc4b9aa-5032-4180-aed2-7c6e32df5469}` |
| Pick points | 1, offset [0, −0.534 mm, +1.300 mm] |
| Status | Registered in the workobject library and in `model_guid_config.json`, but the `3D Target Object Recognition` step is bound to **`t1` only** |

To bring `t2` into production, either switch the recognition step's target object, or add a second recognition branch. Note both parts are the same thickness and similar length — a matching run configured for `t1` may produce **false positives on `t2`** if both are ever present in the same bin.

---

## 9. Mech-Vision Project: `Drill_Tap`

**23 steps.** Continuous-run interval 500 ms. Project file: `MechMaster/Drill_Tap/Drill_Tap.vis`.

### 9.1 Dataflow Graph

```
Capture Images from Camera_1
   ├─[0 Depth Map]──────► 3D Target Object Recognition_1 [0]
   └─[1 Color Image]────► 3D Target Object Recognition_1 [1]

3D Target Object Recognition_1
   ├─[0 Pick Points]────► Validate Poses by Included Angles to Ref Direction_1 [0]
   ├─[1 Pick Point IDs]─► Filter_1 [1]
   ├─[2 Coloured Cloud]─► Path Planning_1 [2]
   ├─[3 Merged Cloud]───► Show Point Clouds and Poses_1 [0]
   ├─[4 Object Cloud]───► Show Point Clouds and Poses_1 [1]
   └─[5 Confidences]────► Filter_4 [1]

Easy Create Poses_1 ──► Validate…Ref Direction_1 [1]      (reference: [0,0,0, 0,1,0,0])

Validate…Ref Direction_1
   ├─[0 bool list]──► Filter_1 [0] ──► Filter_2 [1]
   ├─[0 bool list]──► Filter_4 [0] ──► Filter_3 [1]
   └─[1 valid poses]► Validate Existence of Poses in 3D ROI_1 [0]

Validate Existence of Poses in 3D ROI_1
   ├─[0 bool list]──► Filter_2 [0]  and  Filter_3 [0]
   └─[1 valid poses]► Sort 3D Poses V2_1 [0]

Filter_2 ──► Reorder by Index List_1 [1]       (carries pick-point IDs)
Filter_3 ──► Sort 3D Poses V2_1 [1]            (carries confidences)

Sort 3D Poses V2_1 ─[1 index list]► Reorder by Index List_1 [0]
Sort 3D Poses V2_1 ─[0 sorted poses]► Sort 3D Poses V2_2 [0]
Reorder by Index List_1 ──► Reorder by Index List_2 [1]
Sort 3D Poses V2_2 ─[1 index list]► Reorder by Index List_2 [0]

Sort 3D Poses V2_2 ─[0]► Trim Input List_1  (limit 1) ──┬─► Validate…Ref Direction_2 [0]
                                                        └─► Flip Poses' Axes_1 [0]
Reorder by Index List_2 ──► Trim Input List_2 (limit 1) ──► Path Planning_1 [1]

Easy Create Poses_2 ──► Validate…Ref Direction_2 [1]      (reference: [0,0,0, 1,0,0,0])
Validate…Ref Direction_2 ─[0 bool]► Trigger Control By Flag_1

Flip Poses' Axes_1 ──► Transform Poses_1 ──┬─► Path Planning_1 [0]
                                            └─► Show Point Clouds and Poses_1 [2]

Path Planning_1 ──► Output_1
```

**Control flow:** one control edge — `Trigger Control By Flag_1` gates `Transform Poses_1`.

### 9.2 Step-by-Step Reference

| # | Step name | Type | Key parameters |
|---|---|---|---|
| 1 | `Capture Images from Camera_1` | Capture Images from Camera | Camera `RUM70249B500E015` (NANO ULTRA) @ `192.168.5.10`; param group `drill-tap-cell`; calib group `RUM70249B500E015-EyeToHand-2026-07-17`; `dataTypes=7` (2D + depth + cloud); timeout 10 000 ms; **reconnect attempts 3**; recapture attempts 3; virtual mode off |
| 2 | `3D Target Object Recognition_1` | 3D Target Object Recognition | Target object **`t1`**; procedure `resource/workpiece_recog_v2/{7d8c1e28-…}/t1.json`; debug off |
| 3 | `Validate Poses by Included Angles to Reference Direction_1` | Validate Poses by Included Angle | Axis type 2, **goal axis 4**, threshold **90°**, reference direction (0,0,1), `filterOutAllPoses` off |
| 4 | `Easy Create Poses_1` | Easy Create Poses | `[0,0,0, 0,1,0,0]` — reference pose (180° about X) |
| 5 | `Filter_1` | Filter | Gates pick-point IDs by step 3's boolean list |
| 6 | `Filter_4` | Filter | Gates confidences by step 3's boolean list |
| 7 | `Validate Existence of Poses in 3D ROI_1` | Validate Existence of Poses in 3D ROI | ROI **`Center_line`**; `isOnlyRoi` true; `useRoiInFlange` true; input poses in robot coords |
| 8 | `Filter_3` | Filter | Gates confidences by step 7's boolean list |
| 9 | `Filter_2` | Filter | Gates pick-point IDs by step 7's boolean list |
| 10 | `Sort 3D Poses V2_1` | Sort 3D Poses V2 | **Sort method 8**, divide mode 1, layer interval **0.1 m**, ascending, row dir 0 / col dir 1 |
| 11 | `Reorder by Index List_1` | Reorder by Index List | Re-orders IDs to match step 10's sort |
| 12 | `Sort 3D Poses V2_2` | Sort 3D Poses V2 | **Sort method 5**, divide mode 1, layer interval **0.1 m**, ascending |
| 13 | `Reorder by Index List_2` | Reorder by Index List | Re-orders IDs to match step 12's sort |
| 14 | `Trim Input List_1` | Trim Input List | **numberLimit = 1** — keep best pose only |
| 15 | `Trim Input List_2` | Trim Input List | **numberLimit = 1** — keep matching ID only |
| 16 | `Easy Create Poses_2` | Easy Create Poses | `[0,0,0, 1,0,0,0]` — identity reference |
| 17 | `Validate Poses by Included Angles to Reference Direction_2` | Validate Poses by Included Angle | Same settings as step 3 — final sanity gate on the chosen pose |
| 18 | `Flip Poses' Axes_1` | Flip Poses' Axes | axisType 0, directionType 1, rotate about axis 2 (Z) |
| 19 | `Trigger Control By Flag_1` | Trigger Control By Flag | triggerType 1 — blocks downstream if the final validation fails |
| 20 | `Transform Poses_1` | Transform Poses | transformType 0 |
| 21 | `Path Planning_1` | Path Planning | Scenario 0; cloud in camera coords; `methodToConvertData` 1; target-object cloud search radius 3 mm; auto drift correction **off** |
| 22 | `Show Point Clouds and Poses_1` | Show Point Clouds and Poses | Debug visualisation; bounds −3 mm … 0; normals hidden |
| 23 | `Output_1` | Procedure Out | outputType 2; sends scene point cloud; cloud in camera frame |

### 9.3 Two-Stage Sort — Why

Two `Sort 3D Poses V2` steps run in series with **different sort methods (8 then 5)** on the same 0.1 m layer interval. This is a primary/secondary sort:

1. **Sort 1 (method 8)** groups poses into 100 mm height layers and orders within them.
2. **Sort 2 (method 5)** re-orders the already-layered result by a second criterion.

The net effect is *"pick from the topmost layer first, and within that layer prefer the pose that best satisfies the secondary criterion."* Each sort is paired with a `Reorder by Index List` so the pick-point ID list stays aligned with the pose list — **if you change one sort, you must keep its paired reorder step**, or IDs and poses will desynchronise and the robot will be sent the wrong part's ID.

### 9.4 Regions of Interest

Two 3D ROIs are defined, both stored in `Drill_Tap/resource/3d_roi/`.

| ROI | Used by | Size (mm) | Centre in robot base (mm) |
|---|---|---|---|
| `3d_roi` | Recognition procedure (point-cloud crop, twice) | **1011.9 × 672.9 × 113.1** | (−46.7, 979.9, −136.0) |
| `Center_line` | `Validate Existence of Poses in 3D ROI_1` | **80.0 × 568.1 × 229.1** | (−47.0, 1030.2, −84.6) |

- **`3d_roi`** is the bin volume — everything outside it is discarded before matching. It is only 113 mm tall, which bounds the usable stack height.
- **`Center_line`** is a narrow 80 mm-wide slab running down the middle of the bin. A candidate pose is only accepted if it falls inside this band. This is a deliberate constraint: it forces the robot to pick from the centre of the bin, where the three magnets have the best chance of full contact and where the arm has the most favourable reach and least chance of striking a bin wall.

> ⚠ **`Center_line` is the single most impactful tuning parameter in the project.** Widening it increases the number of candidate picks (fewer "no parts detected" pauses when the bin is nearly empty) but increases the risk of edge picks and bin-wall contact. Narrowing it does the opposite. Change it in small increments and validate with a full bin *and* a nearly empty one.

### 9.5 Recognition Procedure Internals (`t1.json`)

The `3D Target Object Recognition` step wraps a procedure with the following pipeline:

**Point cloud preparation**
1. `Extract 3D Points in 3D ROI` (`3d_roi`) — crop scene cloud
2. `Calc Normals and Estimate Edges of Point Cloud` — search radius 10 mm; max angle between normal and Z axis **70°**; normal variation threshold 10; min edge point count 10; depth difference threshold 5; depth map resolution 1000
3. `Point Filter` (NormalsFilter) — keep points whose normals are within **0–70°** of the −Z alignment vector
4. `Merge Point Clouds`
5. `From Depth Map to Point Cloud`, second `Extract 3D Points in 3D ROI`
6. `Image Filtering` — median filter, kernel size 5

**3D matching**

| Parameter | Value |
|---|---|
| Model selection | `t1` |
| Confidence threshold | **0.3** (surface, edge, and overall) |
| Score strategy | **Ultra-high** |
| Sigma | 0.0069 m ("Large"), 4 update rounds |
| Iteration count | 50 (+ 40 extra-fine iterations) |
| Coarse approach / leaf size | 2 / 4.845 mm |
| Fine leaf size | 7.290 mm |
| Extra-fine leaf size | 3.645 mm |
| Downsampling leaf size | 3.0 mm (`useDownsampling` = false) |
| Voxel length | 4.845 mm (min 0.1 mm, max 15 mm) |
| Angle quantification | 30 |
| Output count | 10 |
| Invisible filter angle | **70°** |
| `considerHoleInModel` | **true** — the bars are perforated; this matters |
| `useLongObjectRefine` | **true**, `longObjectThre` 3 — specifically for high-aspect-ratio parts like these 727 mm bars |
| `useExtraFineMatch` | true |
| `useRemoveOverLappingObjects` / `useRemoveConincidingObjects` | true / true (thresholds 0.3) |
| Expected model point count | 629 |
| Max scene point count | 1 000 000 |

> `useLongObjectRefine` and `considerHoleInModel` are the two settings most specific to this part. A 727 × 71 mm perforated bar is close to degenerate for PPF matching — it is nearly a line, and its hole pattern is the main disambiguating feature. If matching quality drops after a part revision, check that the CAD model's hole pattern still matches the physical part.

### 9.6 Preprocessing Defaults (`resource/preprocess.json`)

| Parameter | Value |
|---|---|
| Cluster tolerance | 3 mm |
| Min / max cluster size | 100 / 3 000 000 |
| Normal variation threshold | 10 |
| Angle range | 0 – 70° |
| Kernel size | 5 |
| Remove noise | disabled |

---

## 10. Path Planning Project

Project: `MechMaster/planning_project/planning_project.json` · Robot model: `yaskawa_hc10.mrob` · Version 2.1.2 · **lite mode enabled**.

### 10.1 Task Sequence

| Order | Task | Type | Description | Key settings |
|---|---|---|---|---|
| 1 | `Branch by Msg_1` | branch_by_msg | Entry branch | size 1, wait timeout 30 s |
| 2 | `Change Tool_1` | tcp | Set robot tool | EE GUID `{608ecf20-…}` |
| 3 | `Fixed-Point Move_2` | move | **Enter-bin point** | Joint target below; vel 1, acc 0.5, blend radius 0.05 |
| 4 | `Visual Recognition_1` | visual_look | Trigger vision | `vision_name = Drill_Tap_Path Planning_1`; eye-in-hand false |
| 5 | `Vision Move_1` | visual_move | **Pick move to vision pose** | maxPlanResultsCount 2; `planFailureOutPort` **true**; 5 interpolation segments; position deviation ≤ 50 mm; angle deviation ≤ 5° (0.0873 rad) |
| 6 | `Relative Move_1` | relative_move | **Approach point** | relative_to 3, avoidance radius 5 mm, blend radius 0.05 |
| 7 | `Relative Move_2` | relative_move | **Retreat point** | relative_to 2, avoidance radius 100 mm, blend radius 0.05 |
| 8 | `Fixed-Point Move_1` | move | **Exit-bin point** | Same joint target as enter-bin |

**Enter-bin / Exit-bin joint target** (identical for both):

| Joint | Radians | Degrees |
|---|---|---|
| J1 | −1.940171 | **−111.16°** |
| J2 | −0.618648 | **−35.45°** |
| J3 | +4.638780 | **+265.78°** |
| J4 | −0.001648 | **−0.09°** |
| J5 | +1.021253 | **+58.51°** |
| J6 | +2.365328 | **+135.52°** |

> Note J3 = +265.78°, which is why the J3 soft limit is set to −5…+355° rather than ±180°. Enter and exit share the same waypoint, so the robot returns to a single known safe posture above the bin between every pick.

### 10.2 Executor Settings

| Parameter | Value | Comment |
|---|---|---|
| `velScale` | **0.2** | Running at 20 % of programmed velocity |
| `accScale` | 1.0 | |
| `check_pcl_collision` | **false** | ⚠ Point-cloud collision checking **disabled** |
| `check_base_pcl_collision` | false | |
| `check_forearm_pcl_collision` | false | |
| `check_upperarm_pcl_collision` | false | |
| `check_wrist_pcl_collision` | false | |
| `workobject_collision` | false | |
| `check_picked_object_with_perception` | false | |
| `octree_or_height_map_resolution_mm` | 2 mm | |
| `heightmap_or_octree` | true (height map) | |
| `singularity_detection_mode` | 1 | |
| `singularity_thre` | 6.2832 (2π) | |
| `joint_index` / `max`/`min_joint_limited_angle` | 4 / ±0.08727 rad (±5°) | Wrist joint constrained to ±5° |
| `safe_bottom_enabled` | false | `safe_bottom_distance_mm` = 5 |
| `robot_service_timeout` | 2000 ms | |
| `end_effector_colliding_volume_threshold` | 5 mm³ | |
| `picked_object_colliding_volume_threshold` | 200 mm³ | |
| `source_of_initial_jps` | 0 (from robot) | Overridden to 1 at runtime by the Standard Interface call |
| `max_saved_vision_record_count` | 10 | |

> **`velScale = 0.2` is a significant finding.** The cell is running at one-fifth of programmed speed. If cycle time is a concern, this is the first place to look — but raise it in small increments and only after adding a tool collision model, because collision checking is currently off.

### 10.3 Output Format

The Path Planning step returns **joint positions (J1–J6, degrees)**, not Cartesian poses. See [§12.1](#121-joint-space-output-instead-of-cartesian).

---

## 11. Robot Communication — Standard Interface

### 11.1 Configuration (`MechMaster/.msoln`)

| Parameter | Value |
|---|---|
| Interface | **TCP Server** |
| Format | **ASCII** |
| Host IP | `0.0.0.0` |
| Port | **50000** |
| Robot type | `YASKAWA_HC10` |
| Communication type | 0 (Standard Interface) |
| `open_interface_service` | true |
| Vision project number | `Drill_Tap` = **1** |
| Planning project | `planning_project` |
| Max poses per send | 20 |
| Vision timeout / Viz timeout | 10 s / 10 s |
| Capture-complete timeout | 10 s |
| `wait_for_capture_complete` | true |
| Euler convention | `sxyz` (XYZ static) |
| Handedness | Right-handed |

The `communication/` folder in the backup is the **stock Mech-Mind Communication Component** source. No custom adapter has been written — the standard TCP ASCII interface is used unmodified.

### 11.2 Observed Protocol Exchange

Captured from `logs/communication_log/logs/center_2026-07-30.html` — 44 messages received and 44 sent: **24 vision triggers (command 101) and 20 result retrievals (command 105)**, i.e. 20 complete pick cycles.

**Step 1 — Robot triggers vision (command 101):**

```
Robot → IPC:   101,1,0,2,450.628000,720.155000,140.928000,-179.762000,-0.127100,24.421900\r
IPC  → Robot:  101,1102\r
```

| Field | Meaning |
|---|---|
| `101` | Start Mech-Vision project |
| `1` | Vision project number (`Drill_Tap`) |
| `0` | Robot pose type |
| `2` | Expected number of vision points |
| `450.628, 720.155, 140.928` | Flange position X, Y, Z (mm) |
| `-179.762, -0.127, 24.422` | Flange orientation Rx, Ry, Rz (deg) |
| `1102` | Status: project triggered successfully |

**Step 2 — Robot retrieves the plan (command 105):**

```
Robot → IPC:   105,1,1\r
IPC  → Robot:  105,1103,1,1,1,-89.834,-58.0994,239.947,-1.0087,62.3244,112.2825,0,0\r
```

| Field | Meaning |
|---|---|
| `105` | Get vision result |
| `1` | Project number |
| `1` | Data type — **1 = joint positions** |
| `1103` | Status: vision result sent |
| `1,1,1` | Pose count, index, label count |
| `-89.834 … 112.2825` | **J1–J6 in degrees** |
| `0,0` | Label / trailing fields |

### 11.3 Sample Pick Poses

Ten consecutive picks from the 2026-07-30 session, showing the working joint envelope:

| # | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| 1 | −89.834 | −58.099 | 239.947 | −1.009 | 62.324 | 112.283 |
| 2 | −77.109 | −57.262 | 243.622 | 1.396 | 60.097 | 100.839 |
| 3 | −71.077 | −60.122 | 237.005 | 1.281 | 63.931 | 92.884 |
| 4 | −83.556 | −58.965 | 239.008 | −0.338 | 63.071 | 105.909 |
| 5 | −66.062 | −61.578 | 233.981 | 1.649 | 65.797 | 88.377 |
| 6 | −89.615 | −57.953 | 241.322 | −0.946 | 61.036 | 114.256 |
| 7 | −71.117 | −60.410 | 237.212 | 1.017 | 63.323 | 92.941 |
| 8 | −83.450 | −58.908 | 240.192 | −0.166 | 61.933 | 105.853 |
| 9 | −65.113 | −62.435 | 234.007 | 4.417 | 66.262 | 86.108 |
| 10 | −92.803 | −57.787 | 240.630 | −9.644 | 62.392 | 122.886 |

**Observed envelope:** J1 −93° … −65°, J2 −62° … −57°, J3 +234° … +244°, J4 −10° … +4°, J5 +60° … +66°, J6 +86° … +123°.

The tight J2/J5 spread confirms a consistent top-down pick posture; J1/J6 vary with the bar's position along the bin.

### 11.4 Robot-Side Program

*TBD* — job/program name on the YRC1000micro and the MM job-set version in use.

---

## 12. Design Decisions and Rationale

This section records **why** the cell is built the way it is. These are the details most likely to be lost and most expensive to rediscover.

### 12.1 Joint-Space Output Instead of Cartesian

**Decision:** Path Planning returns **joint positions (J1–J6)**, not Cartesian (X, Y, Z, Rx, Ry, Rz) poses.

**Reason:** When Cartesian targets were sent, the **robot controller miscalculated the inverse kinematics** for the poses required to reach into the bin — producing unreachable-target errors or the wrong arm configuration. The HC10's pick posture puts J3 beyond +234°, in a region where multiple IK solutions exist and the controller's solution selection did not agree with the planner's.

**Consequence:** Mech-Vision's planner resolves IK itself and hands the controller explicit joint angles, removing the ambiguity entirely.

**Implications for anyone modifying this cell:**

- The J3 soft limit **must** remain −5° … +355°. Restoring a conventional ±180° limit will break every pick.
- The robot job must consume **command 105 with data type 1** (joint positions). Switching to a Cartesian data type will reintroduce the original fault.
- Because targets are joint-space, they are **only valid for this exact robot mounting and calibration**. Moving the robot base or re-calibrating invalidates the stored fixed waypoints in [§10.1](#101-task-sequence).

### 12.2 Printed Calibration Board Taped Into the Bin

**Decision:** OCB-20 pattern printed true-to-size and taped to the bin floor instead of using a rigid board.

**Reason:** Calibrating in the exact plane the parts occupy, at zero cost, without fixturing a rigid board inside a bin the AMR carries away.

**Trade-off:** No residual error metric, and paper is dimensionally sensitive. See [§6.4](#64-caveats-and-re-calibration-guidance).

### 12.3 Centre-Line Pick Constraint

**Decision:** A narrow 80 mm `Center_line` ROI restricts valid picks to the middle of the bin.

**Reason:** Three spring-mounted magnets need good face contact; edge picks risk partial contact, tilted lifts, and bin-wall collisions — particularly relevant because collision checking is off.

**Trade-off:** Parts that settle against the bin walls are not pickable and must be nudged toward the centre, which contributes to the "no parts detected" pause when the bin runs low.

### 12.4 Single-Pick Trimming

**Decision:** Both `Trim Input List` steps are set to **1**.

**Reason:** One part is planned and delivered per vision cycle. The robot re-triggers vision for every pick rather than caching a queue of poses. This is slower than batch picking, but every pick is planned against a **fresh point cloud** — critical when removing a bar disturbs the ones beneath it.

### 12.5 Fast Fringe Coding

**Decision:** `ProjectorFringeCodingMode = 0` (Fast) with high surface smoothing.

**Reason:** Large flat matte-finish parts do not need Accurate-mode detail; Fast mode keeps vision execution under ~2 s.

---

## 13. Operating Procedure

### 13.1 Normal Start-Up

1. **Power on** the IPC and confirm Mech-Vision, the Path Planning project, and the Communication Component are running.
2. Confirm the camera is connected — camera `RUM70249B500E015` should show `connected: true` at `192.168.5.10`.
   - **Allow ~30 minutes of camera warm-up** before running production if pick accuracy matters. This is the manufacturer's recommended warm-up for the NANO ULTRA to reach its stated accuracy. A cold camera drifts, and that drift lands directly on the pick pose. Mech-Eye Viewer includes a Warm-Up Tool for this.
3. Confirm the Standard Interface is listening on port **50000**.
4. Power on the robot controller; clear any alarms; verify the robot is at or near the enter-bin posture.
5. **Verify a pick bin is present** under the camera and contains parts.
6. Verify the CNC is powered, homed, and the fixture is clear.
7. **Start the robot job** on the YRC1000micro pendant.

> **Everything is driven from the robot side.** Once a bin with parts is in position, starting the robot program is the only operator action required. There is no separate HMI start on the vision side.

### 13.2 Normal Operation

The robot cycles autonomously: trigger vision → receive joint plan → enter bin → pick → load CNC → drill/tap → unload → repeat.

### 13.3 Bin Changeover

1. The robot pauses when no parts are detected.
2. The Pudu AMR removes the empty bin and delivers a full one.
3. Once the bin is in position, restart the robot job.

> ⚠ **There is currently no signal between the Pudu and the cell.** The robot cannot know whether a bin is present, absent, or mid-exchange. Bin changeover is a **manual, supervised step** today. See [§17](#17-roadmap).

### 13.4 Normal Shutdown

1. Allow the current pick-and-load cycle to finish.
2. Stop the robot job at the enter-bin waypoint.
3. Leave Mech-Vision and the Communication Component running, or close the solution cleanly if powering down the IPC.

---

## 14. Performance Data

### 14.1 Vision Execution Time

From `logs/vision_log/logs/2026-07-30.log` — **367 measured executions**:

| Metric | Value |
|---|---|
| Mean | **1.826 s** |
| Minimum | 0.630 s |
| Maximum | 2.571 s |
| Typical steady-state | 1.89 – 1.94 s |

Independently confirmed on 2026-08-12: **1.50045 s** for a 37-step-execution run.

### 14.2 End-to-End Vision Response

Time from robot command `101` to receipt of the `105` reply, measured over 20 cycles: **minimum 2.73 s**, typical ~3 s. The delta over raw vision time is path planning plus interface round-trip.

### 14.3 Reliability

| Metric | Value |
|---|---|
| Vision completions logged (2026-07-30) | 182 |
| Error codes other than `CV-E0000` (success) | **0** |
| Planning exit codes | `MP-E0000` (success) |

### 14.4 Not Yet Measured

- **Full cell cycle time** (pick → CNC load → drill/tap → unload) — *TBD*
- **Parts per hour** — *TBD*
- **Parts per bin** — *TBD*
- **Mis-pick rate** — reported anecdotally as very rare; not instrumented

> With `velScale` at 0.2, the robot motion — not the vision — almost certainly dominates cycle time. Measuring the full cycle should be the first step before any optimisation.

---

## 15. Known Issues and Troubleshooting

### 15.1 No Parts Detected → Robot Pauses

**Symptom:** Robot stops; Mech-Vision log shows a warning cascade:

```
[W] 3D Target Object Recognition_1: No recognition result.
[W] Validate Poses by Included Angles to Reference Direction_1: The 1th input is empty!
[W] … : No output. Auto fill output.
[W] Path Planning_1: No output. Auto fill output.
```

**This is normal, designed behaviour** when the bin is empty. Every downstream step reports "No output. Auto fill output." — a cascade, not multiple independent faults. **Read the first warning only**; the rest are consequences.

**Response:**

1. Confirm the bin is genuinely empty.
2. If parts remain, check they are not all outside the `Center_line` ROI (i.e. pushed against the walls). Nudge them toward the centre.
3. Confirm parts lie within the 113 mm-tall `3d_roi` volume.
4. Confirm the bin is correctly positioned under the camera.

### 15.2 False Pick Pose With 1–2 Parts Remaining

**Symptom:** On rare occasions, when only 1–2 parts remain in the bin, the detected pick pose is slightly off, which can result in a mis-placed part at the CNC fixture.

**Frequency:** Very rare — the integrator reports this is uncommon even in the low-part condition that triggers it.

**Likely mechanism:** With a nearly empty bin, a lone bar can sit at an angle against the bin floor or wall, presenting a partial surface to the camera. Two factors compound this:

- With a `confidenceThreshold` of 0.3 and a `Center_line` band only 80 mm wide, a marginal match can pass validation.
- The 727 mm bar spans almost the full camera field of view (~740 mm at the working distance — see [§3.2.1](#321--field-of-view-is-the-binding-constraint)). A bar lying diagonally can put one end outside the frame, so the match is made against a **truncated point cloud**. A partial cloud of a long, near-symmetric part is exactly the condition under which PPF matching slips along the part's length.

The three-magnet EOAT then grips at a slightly wrong point, and the offset carries through to placement.

The second factor also explains why the fault is specific to a nearly empty bin: with a full bin the bars are constrained by their neighbours and lie roughly parallel to the bin; the last one or two are free to rotate.

**Mitigations, in order of preference:**

1. **Do not run bins to empty** — swap at 2–3 parts remaining. Simplest and most effective.
2. Raise `confidenceThreshold` above 0.3 in the 3D matching step. Reduces false matches; may increase "no parts detected" pauses.
3. Verify the bin's lateral centring under the camera. Because the part nearly fills the FOV, a small bin offset costs disproportionate coverage.
4. Add a part-present or part-position check at the CNC fixture to catch a bad placement before the drill cycle.
5. Add a collision model to the EOAT and re-enable point-cloud collision checking.

### 15.3 Robot Cannot Reach a Detected Pose

**Symptom:** Vision succeeds, planning fails; `Vision Move_1` exits via its failure port (`planFailureOutPort` is enabled).

**Checks:**

1. Verify the joint soft limits are unchanged — especially **J3 = −5° to +355°**.
2. Confirm the robot job is requesting **data type 1 (joint positions)** on command 105 — a Cartesian request will reintroduce the original IK fault ([§12.1](#121-joint-space-output-instead-of-cartesian)).
3. Check the pose is inside `Center_line`.
4. `maxPlanResultsCount` is 2 — only two plan candidates are attempted per pose.

### 15.4 Camera Disconnect

**Symptom:** `Camera 192.168.5.10 get disconnected` in the Communication Component log.

**Observed:** Once on 2026-07-30 (10:12:05), self-recovered in 1 second.

**Checks:** camera Ethernet cable and strain relief at the extrusion mount; 24 V supply; IPC NIC link. The capture step retries **3 times** before failing, which covers brief drops. Recurrence indicates a physical connection issue.

### 15.5 Diagnostic Log Locations

| Log | Path |
|---|---|
| Vision execution | `MechMaster/logs/vision_log/logs/YYYY-MM-DD.log` |
| Camera | `MechMaster/logs/vision_log/logs/camera/camera_YYYY-MM-DD.log` |
| Camera performance | `MechMaster/logs/vision_log/logs/camera/camera_performance_YYYY-MM-DD.log` |
| Communication / interface | `MechMaster/logs/communication_log/logs/center_YYYY-MM-DD.html` |
| Planning history | `MechMaster/logs/viz_log/plan_history/YYYY-MM-DD/*.planning` |
| Operation | `MechMaster/logs/vision_log/logs/Operation_YYYY-MM-DD.log` |

The planning history retains every planning attempt with a timestamp — invaluable for reconstructing an intermittent fault after the fact.

---

## 16. Maintenance

### 16.1 Routine

| Interval | Task |
|---|---|
| Daily | Wipe the camera lens/protective glass. Dust and coolant mist directly degrade point-cloud quality. |
| Daily | Verify the magnet faces are clean and free of chips — steel swarf on a magnet face causes standoff and dropped parts. |
| Weekly | Inspect EOAT springs for fatigue, permanent set, or cracking. |
| Weekly | Check camera mount rigidity on the extrusion. **Any movement invalidates calibration.** |
| Monthly | Verify pick accuracy against a known part position; re-calibrate if drifting. |
| Monthly | Archive and clear logs (`logs/` grows continuously). |
| As needed | Back up the full solution folder — see [§16.2](#162-backup). |

### 16.2 Backup

Back up the entire `MechMaster/` solution folder. The reference backup for this document is `MechMaster_drill_Tap_Backup_7_30_2026.zip` (2026-07-30).

**Take a fresh backup after any change to:** camera parameters, ROIs, matching parameters, calibration, path-planning waypoints, or target-object models.

> ⚠ **`Drill_Tap.vis` is a binary file and cannot be diffed or read outside Mech-Vision.** The companion `Drill_Tap.json.bak` is human-readable, but it is only written at save time and may lag the live project. Treat backups as whole-folder snapshots and date them — do not rely on being able to inspect a `.vis` after the fact.

### 16.3 After Re-Calibration

Re-calibration changes the camera-to-base transform, which invalidates:

- the stored **fixed joint waypoints** in the planning project ([§10.1](#101-task-sequence)),
- both **3D ROIs**, which are stored in both camera and robot coordinates.

Re-teach and re-verify both after any calibration change.

---

## 17. Roadmap

### 17.1 Pudu ↔ Cell Integration (Near Term)

**Current state:** No electrical or network connection between the Pudu AMR and the cell. The robot has no way to know whether a bin is present.

**Planned:** A bin-present signal so the cell can:

- confirm a bin is in position before attempting a pick,
- request a bin swap automatically when parts run out,
- resume without operator intervention.

**Implementation notes:** The IPC's fourth NIC is factory-labelled **ROBOT / PLC** and is currently unused — the natural home for this integration. The Communication Component already supports Modbus TCP Slave, EtherNet/IP, PROFINET, and Siemens S7 client alongside the TCP server in use, so a second interface can be added without disturbing the working robot link.

### 17.2 Additional Part Numbers

`t2` (`blender_modified_MM_Drill_Tap_wing_peice`, 735.94 × 87.01 × 7.94 mm) is already modelled, has a pick point defined, and is registered in `model_guid_config.json` — but the recognition step is bound to `t1` only.

**To productionise `t2`:**

1. Decide between a single recognition step with a switchable target object, or parallel recognition branches with a part-select input from the robot or PLC.
2. Re-verify the `Center_line` ROI — `t2` is 16 mm wider, so centre-line clearance differs.
3. Confirm the CNC fixture and program for `t2`.
4. **Test for cross-matching**: `t1` and `t2` are the same thickness and similar length. Verify a `t1`-configured matching run does not false-positive on a `t2`, and vice versa.

Smaller AMTE 400 components are planned beyond `t2`. Note that the current `3d_roi` is only 113 mm tall and the `Center_line` band 80 mm wide — both were sized for a 727 mm bar and will need revisiting for materially different geometry.

**Smaller parts are the easy direction; longer parts are not.** Anything appreciably longer than `t1` will not fit the camera's ~740 × 525 mm field of view at the current mounting height ([§3.2.1](#321--field-of-view-is-the-binding-constraint)), and raising the camera pushes past the NANO ULTRA-700M's 800 mm working-distance limit. Plan part families accordingly.

### 17.3 Recommended Improvements

| Priority | Improvement | Rationale |
|---|---|---|
| High | Add an EOAT collision model and enable point-cloud collision checking | Currently the cell has no collision protection beyond waypoint design ([§5.2](#52--open-risk--no-tool-collision-model)) |
| High | Instrument full cycle time and parts-per-hour | No throughput baseline exists ([§14.4](#144-not-yet-measured)) |
| Medium | Raise `velScale` above 0.2 | Likely the largest single cycle-time gain — but only after collision checking is restored ([§10.2](#102-executor-settings)) |
| Medium | Re-calibrate with a rigid board and record a residual error figure | Current 3-point paper calibration has no error metric ([§6.4](#64-caveats-and-re-calibration-guidance)) |
| Medium | Part-present sensing at the CNC fixture | Catches the rare mis-placement before the drill cycle ([§15.2](#152-false-pick-pose-with-12-parts-remaining)) |
| Low | Automated log rotation/archival | `logs/` grows without bound |

---

## 18. File and Directory Reference

```
MechMaster/
├── .msoln                                  ← Solution config: interface, port, robot type
├── Drill_Tap/                              ← Mech-Vision project
│   ├── Drill_Tap.vis                       ← LIVE project (binary — not human-readable)
│   ├── Drill_Tap.json.bak                  ← Readable backup (may lag the .vis)
│   ├── calibration/
│   │   ├── RUM70249B500E015-EyeToHand-2026-07-17/
│   │   │   ├── calibSetting.json           ← Board type, pattern, method
│   │   │   ├── calib_data.json             ← The 3 touch points
│   │   │   ├── extri_param.json            ← Camera pose in robot base
│   │   │   ├── intri_param.json            ← Camera intrinsics
│   │   │   ├── error_points_loc.json
│   │   │   ├── Accurate_error_distribution.pcd
│   │   │   └── calib_images/EyeToHand/
│   │   └── RUM70249B500E015/               ← Virtual/default calibration group
│   ├── config/
│   │   ├── data_storage_config.json
│   │   └── visualization_config.json
│   └── resource/
│       ├── 3d_roi/
│       │   ├── 3d_roi.json                 ← Bin volume ROI
│       │   └── Center_line.json            ← Centre-band pick constraint
│       ├── preprocess.json
│       └── workpiece_recog_v2/{7d8c1e28-…}/
│           ├── t1.json                     ← Recognition procedure for t1
│           ├── t2.json                     ← Recognition procedure for t2
│           ├── advanced_config.json
│           ├── model_guid_config.json
│           └── template.json
├── planning_project/
│   ├── planning_project.json               ← Task sequence, waypoints, executor settings
│   └── yaskawa_hc10.mrob                   ← Robot kinematic model
├── Mech-Viz-YMfUNP/                        ← Viz container (no authored workcell program)
├── resource/
│   ├── scanParameterGroup/
│   │   └── RUM70249B500E015-scanParameterGroup.json   ← All 9 camera param groups
│   ├── tool_library/
│   │   ├── tool_config.json                ← TCP definition
│   │   └── model_affines.json              (empty)
│   ├── workobject_library/
│   │   ├── t1/  ├── config.json            ← Pick point, symmetry, relaxation
│   │   │        └── models/  ├── LSR-M4-8_E Support Copy.stl
│   │   │                     ├── surface.ply / edge.ply / solid.ply / weight.ply
│   │   │                     └── collision/t1.binvox
│   │   └── t2/  (same structure — blender_modified_MM_Drill_Tap_wing_peice.stl)
│   └── yaskawa_hc10.mrob
├── communication/                          ← Stock Communication Component (unmodified)
└── logs/                                   ← vision_log, communication_log, viz_log
```

---

## 19. Open Items

Items not yet captured. Fill these in and re-issue.

| # | Item | Section |
|---|---|---|
| 0 | **Confirm the camera variant** — NANO ULTRA-**350M** or **700M**. The 761 mm working distance in use is only valid for the 700M (400–800 mm); if a 350M is fitted it is operating well outside its 250–500 mm range, which would explain accuracy problems. Check the label on the camera body. | §3.2 |
| 1 | IPC IP addresses on each NIC; which physical port serves camera vs. robot | §4 |
| 2 | Robot job/program name on the YRC1000micro; MM job-set version | §11.4 |
| 3 | EOAT magnet voltage, current, and robot DO mapping | §5 |
| 4 | Whether any part-present feedback exists on the EOAT | §5 |
| 5 | CNC drill/tap machine make and model | §3.4 |
| 6 | CNC cycle-complete handshake — I/O signal or fixed dwell? | §13.2 |
| 7 | AMTE 400 part number / drawing reference for `t1` and `t2` | §8 |
| 8 | Parts per bin; bin dimensions | §14.4 |
| 9 | Full cell cycle time and parts per hour | §14.4 |
| 10 | Pudu AMR model and bin drop-off repeatability spec | §3.4, §6.4 |
| 11 | Mech-Eye SDK version and camera firmware version | §2.2 |
| 12 | Commissioning date; others who worked on the cell | §20 |
| 13 | Safety assessment / risk assessment reference for cobot operation | — |

---

## 20. Revision History

| Rev | Date | Author | Change |
|---|---|---|---|
| A | 2026-08-12 | Vedansh (documented with Claude) | Initial as-built documentation from solution backup `MechMaster_drill_Tap_Backup_7_30_2026.zip` (2026-07-30), site photographs, and integrator interview |

---

## Appendix A — Source Material

This document was compiled from:

- **Solution backup:** `MechMaster_drill_Tap_Backup_7_30_2026.zip` — solution timestamp 2026-07-30 18:49:49
- **Runtime logs:** vision (367 executions), communication (44 interface messages / 22 cycles), and planning history for 2026-07-29 and 2026-07-30
- **Site photographs:** cell overview, pick bins with parts, IPC port panel, camera internals
- **Mech-Vision screenshots:** 2026-08-12, showing the 23-step project graph and a 1.50 s execution
- **Mech-Eye Industrial 3D Camera User Manual** v2.5.4
- **Mech-Mind IPC STD documentation**
- **Integrator interview** — Vedansh, 2026-08-12

### A.1 Verification Notes

- All parameter values are read directly from the solution backup, not transcribed from screenshots.
- Part dimensions are computed from the STL mesh bounding boxes.
- Camera pose and ROI dimensions are converted from the stored quaternion/half-length representations.
- Joint values are converted from stored radians to degrees.
- **Caveat:** `Drill_Tap.json.bak` (2026-07-28) was used for step parameters because `Drill_Tap.vis` (2026-07-30) is binary. The step count and topology match the 2026-08-12 screenshots exactly (23 steps), so the structure is current; however, **any parameter changed between 2026-07-28 and 2026-07-30 would not be reflected here.** Verify parameters in the live project before relying on them for a change.
- Items marked *TBD* were not derivable from any source and were deliberately left blank rather than estimated.
