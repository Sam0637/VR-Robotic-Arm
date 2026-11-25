Controller control

Scripts: Assets/Scripts/Controller/ControllerArmDriver.cs and Assets/Scripts/Robot/GripperGraspAssist.cs


Controls

Move the controller and the gripper follows the same movement.
Hold the trigger to close the gripper, release it to open the gripper.
Press A (X on the left controller) to re-centre: the arm stays where it is and the current position of the controller becomes the new reference.
The motion is relative, so the arm does not jump to the controller when control starts.
Without a headset, Space works as the trigger and R as re-centre.


Setup

1. Select niryo_one in the Hierarchy.
2. Add Component > ControllerArmDriver. ArmIKFollower, RobotArmController, GripperController and GripperGraspAssist are added with it.
3. Drag the Right Controller of the XR Origin into the Controller field. If it is left empty, the script looks for an object named Right Controller or creates one.
4. In Project Settings > XR Plug-in Management > OpenXR, add the Oculus Touch Controller Profile to the interaction profiles, otherwise the controllers are not detected.


Settings

Motion Scale: 1 means the same amplitude as the hand, lower is more precise.
Smoothing and Dead Zone: reduce shaking.
Follow Controller Rotation: off by default, the gripper points down.
Hold Grip To Move: the arm only moves while the side button is held.
Max Reach, Min Reach, Min Height: workspace limits from the robot base.
Close Threshold and Open Threshold: trigger values that close and open the gripper.


Grasping

When the gripper closes, GripperGraspAssist attaches the nearest object between the fingers (within Grab Radius) so it does not slip. The object is released when the gripper opens. A part held by the gripper is only counted by a sorting bin once it has been released.


Mixed reality

The controls work the same way in VR and in mixed reality. For passthrough, enable Meta Quest: Camera (Passthrough) in the OpenXR settings, add an AR Session to the scene, add an AR Camera Manager to the camera and set its background to black with alpha 0.
