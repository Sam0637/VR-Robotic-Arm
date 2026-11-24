# Production line (conveyor + sorting)

Scripts in `Project/Robotic Arm/Assets/Scripts/ProductionLine/`, namespace `VRRoboticArm.ProductionLine`.

## Scenario

Coloured parts (red, green, blue) arrive on a conveyor in front of the robot. The belt stops when a part reaches the robot. The user makes the robot pick the part and drop it in the bin of the same colour. A score board counts correctly sorted, wrongly sorted and missed parts.

## Scripts

| Script | Role |
|---|---|
| `ProductionLineBuilder.cs` | Builds everything on Play: solid pedestal under the robot, conveyor, spawner, stop zone, 3 bins, score board, simple factory hall. |
| `ConveyorBelt.cs` | Moves the parts lying on the belt (kinematic Rigidbody trick: the belt is moved back, then forward with `MovePosition`, so friction drags the parts). |
| `PartSpawner.cs` | Drops a part of random colour every few seconds (never too close to the previous one). |
| `ProductionPart.cs` | Colour of a part (`PartColor`), removed if it falls far. |
| `ConveyorStopZone.cs` | Trigger in front of the robot: the belt stops while a part is inside. |
| `SortingBin.cs` | Trigger inside a bin: a part staying 0.5 s is counted (right or wrong colour). |
| `MissedPartZone.cs` | Trigger at the end of the belt: the part was missed. |
| `ProductionScore.cs` | Counters, shown on a 3D text (visible in VR). |

## Setup

1. Open a scene containing the robot (e.g. `RobotTestImport`).
2. **GameObject → Create Empty**, name it `Production Line`, keep it at position 0, rotation 0, scale 1.
3. **Add Component → ProductionLineBuilder**. Select it: yellow boxes in the Scene view preview the belt and the bins.
4. Press Play.

All positions are given in the robot frame (metres from the robot base: x in front, y to the robot's left, z up), so the line stays within reach if the robot is moved. The layout was checked with the IK model: the pick-up point on the belt and the three bins are reachable with the gripper pointing down.

## Notes

- The builder locks the robot base (`ArticulationBody.immovable`) so the robot cannot be pushed away from the line.
- The factory hall is a simple placeholder (floor, walls, pillars, safety lines). Untick **Create Factory Hall** when a factory asset pack is imported.
- Part size is 3 cm because the Niryo gripper opening is small; picking itself (attaching the part to the gripper) is a separate task.
- Not yet run inside Unity: the scripts compile against the Unity API signatures, but sizes, speed and friction may need tuning.
