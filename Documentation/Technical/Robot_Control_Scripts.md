# Robot control scripts (Niryo One)

Scripts in `Project/Robotic Arm/Assets/Scripts/Robot/`, namespace `VRRoboticArm.Robot`.

| Script | Role |
|---|---|
| `NiryoOneKinematics.cs` | Forward and inverse kinematics of the 6 arm joints, built from `niryo_one.urdf`. Pure C#, no Unity dependency. |
| `RobotArmController.cs` | Configures the 6 ArticulationBody drives and moves them to the requested angles (speed-limited). Disables the URDF Importer keyboard `Controller`. |
| `ArmIKFollower.cs` | Each physics step, solves the IK so the gripper follows a target Transform, and sends the angles to `RobotArmController`. |
| `GripperController.cs` | Opens / closes the gripper (`Open()`, `Close()`, `SetOpenAmount(0..1)`). |

## Setup

1. Open `Scenes/RobotTestImport`.
2. Select the robot root `niryo_one` in the Hierarchy.
3. **Add Component** → `ArmIKFollower` (this also adds `RobotArmController`), then **Add Component** → `GripperController`.
4. Press Play. A small sphere "IK Target (test)" appears at the gripper: select it and move it in the Scene view with the Move tool. The arm follows it.
5. Gripper: change **Open Amount** in the Inspector during Play (1 = open, 0 = closed).

For hand tracking, assign the object moved by the hand to the **Target** field of `ArmIKFollower`, and call `GripperController.Open()` / `Close()` from the pinch detection.

## How it works

- The IK uses damped least squares (Jacobian computed numerically), warm-started from the previous solution, with the URDF joint limits.
- Frames: the math is done in the ROS frame of `base_link`. The URDF Importer maps ROS `(x, y, z)` to Unity `(-y, z, x)`.
- Orientation modes: `PositionOnly`, `PointDown` (default, for picking objects on a table), `MatchTargetForward` (the gripper points along the target's forward axis).
- Joint angles sent to the drives = URDF angles in degrees (same convention as Unity's Niryo One pick-and-place tutorial).

## What was checked, and what still needs testing in Unity

- Checked outside Unity: the forward kinematics reproduces the positions of every link stored in `RobotTestImport.unity` (difference < 0.1 mm), and the IK reaches 300 random targets out of 300 (position error < 5 mm).
- The C# compiles against the Unity API signatures, but **has not been run inside Unity yet**.
- On Play, `ArmIKFollower` logs a self-check after 2 s ("Self-check OK" or a warning if the model and the simulated robot differ).
- To tune in Unity: `toolOffset` (0.06 m is an estimate of the distance to the middle of the fingers), and the gripper open/closed positions (taken from Unity's pick-and-place tutorial).
