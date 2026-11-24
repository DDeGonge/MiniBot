# MiniBot - GitHub Issue Backlog

Generated from [ROADMAP.md](./ROADMAP.md) (roadmap last updated 2026-09-08).
One heading = one issue. Paste the heading text as the issue title and the paragraph below it as the body.

28 issues total: 5 CAD, 4 PCB, 4 MiniBot firmware, 2 Server firmware, 13 Application.

## Suggested labels

Create these first so the area/type filters work:

| Kind | Labels |
|------|--------|
| Area | `area:CAD`, `area:PCB`, `area:Bot Firmware`, `area:Server Firmware`, `area:Application` |
| Type | `short-term`, `next-rev`, `spike` |

---

## Area: CAD (OnShape)

**OnShape model:** https://cad.onshape.com/documents/4f8eaef75458146767928ab5/w/f159d6d65b9091531e1ead34/e/13b51722310f27710438c727

### CAD: Finish the clock assembly

Complete the clock design in the OnShape assembly. It is currently unfinished and blocks the short-term CAD milestone.

Labels: `area:CAD`, `short-term`

### CAD: Finish the board

The physical board enclosure and frame are unfinished in the OnShape model. Complete it so piece topper geometry and electromagnet mount positions can be finalized against a finished reference.

Labels: `area:CAD`, `short-term`

### CAD: Finish the piece toppers

Remaining chess piece topper models are incomplete. Finish all 12 unique topper designs so they can be printed and mounted on the bot chassis.

Labels: `area:CAD`, `short-term`

### CAD: Strengthen the topper retainer tabs

The topper retainer tabs are too weak and fail in service. Reinforcing them requires changing the PCB outline, so this must be coordinated with a matching revision of the MiniBot Mainboard PCB. Please open a paired PCB issue and block the two on each other.

Labels: `area:CAD`, `next-rev`

### CAD: Make wheel protrusion repeatable and adjustable

Wheel / o-ring protrusion off the chassis varies from build to build, which affects ground contact and drive geometry. Design a positive stop or locking mechanism that fixes wheel position repeatably, ideally user-adjustable so tire diameter and ride height can be tuned without a redesign.

Labels: `area:CAD`, `next-rev`

---

## Area: PCBs

Board designs live in `pcbs/`: `pcbs/MiniBot_MainBoard/`, `pcbs/Server_Motherboard/`, `pcbs/Charge_Case/`.
Gerbers are in the `GERBERS/` subfolder of each.

### PCB: MiniBot Mainboard - fix charge status LED polarity

The charge-status indicator on the MiniBot Mainboard is inverted: it reads charged when uncharged (or vice versa). Swap the polarity of the LED driver in the schematic. Self-contained fix, no layout changes.

Labels: `area:PCB`

### PCB: MiniBot Mainboard - move pogo pin contacts to the board edge

Relocate the charging pogo pin contact pads from their current interior position out to the board edge, so the bot sits flush against the Charge Case fixture. Requires re-measuring contact spacing against the agreed charge fixture from the Charge Case issue.

Labels: `area:PCB`, blocked by: `Charge Case - finalize pogo pin charge layout`

### PCB: MiniBot Mainboard - remove obsolete pogo pin pads

Once the pogo pin contacts have moved to the board edge, delete the dead pads left over from the previous interior layout. Removes unused components and avoids stray solder / contact hardware in the board field.

Labels: `area:PCB`, blocked by: `MiniBot Mainboard - move pogo pin contacts to the board edge`

### PCB: Charge Case - finalize pogo pin charge layout and method

Still a large TBD. Decide and document the full charging interface: pogo pin count, spacing, angle and travel; matching board-edge contact geometry on the MiniBot Mainboard; retention method (magnets, screws, gravity, or case lip); and the bot-to-case alignment tolerance. This decision blocks the Mainboard contact move and the chassis wheel design.

Labels: `area:PCB`, `spike`

> No issues filed for `pcbs/Server_Motherboard/` - the roadmap states "No changes planned."

---

## Area: MiniBot firmware (`firmware/MiniBot_StepperClient`)

ESP32-C3 per-bot firmware. Tasks created in `src/main.cpp`, constants in `include/config.h`.

### StepperClient: fine-tune the recently added changes

Several recently landed subsystems are implemented but have not been tuned or validated across the full 34-bot fleet: `ota_updater.cpp`, radio duty cycling in `esp_now_communicator.cpp`, and the arc motion path in `robot.cpp`. Review each one, exercise it on hardware, and tune the relevant constants in `include/config.h`.

Labels: `area:Bot Firmware`, `short-term`

### StepperClient: confirm light sleep is actually engaging

Verify that ESP32-C3 light sleep enters and resumes correctly alongside ESP-NOW, the MMC5633 magnetometer reads, and the FreeRTOS tasks. Confirm with measured battery drain over an idle period. Also confirm that `BatteryMonitor`'s task suspension path cannot miss a wake-up event.

Labels: `area:Bot Firmware`, `short-term`

### StepperClient: gate position sensing on demand

`PositionEstimator_Sensor` reads the MMC5633 at high rate continuously, which is expensive on a ~170 mAh pack. Add logic to enable and disable the sensor task intelligently - only when a piece is in motion, when pose confidence is low, or when the Server explicitly requests a field measurement. Keep this decision consistent with the radio duty-cycling state so a bot that needs sensing always has its radio on.

Labels: `area:Bot Firmware`

### StepperClient: switch motion to discrete primitives (linear / arc / rotate in place)

`robot.cpp` already contains arc geometry (`MIN_ARC_RADIUS_MM`, inner and outer wheel distances), but the system still drives motion as an implicit two-phase rotate-then-translate. Add explicit primitive types to `MotionCommand` / `messages_ipc.h` so the Coordinator can request a straight line, a constant-radius arc, or a rotate-in-place independently. This is what lets the planner produce realistic differential-drive paths.

Labels: `area:Bot Firmware`, `area:Application`

<!-- APPEND-2 -->

