using UnityEngine;

namespace VRRoboticArm.Robot
{
    /// <summary>
    /// Makes the Niryo One gripper follow a target Transform using inverse kinematics.
    ///
    /// Put this on the robot root (next to RobotArmController). If no target is assigned,
    /// a small sphere is created at the gripper when Play starts: select it in the Scene view
    /// and move it with the Move tool to test the arm without a VR headset.
    /// Later, the hand-tracking script only has to move this same target.
    /// </summary>
    [RequireComponent(typeof(RobotArmController))]
    public class ArmIKFollower : MonoBehaviour
    {
        public enum OrientationMode
        {
            /// <summary>Only the position is followed; the wrist is free.</summary>
            PositionOnly,
            /// <summary>The gripper tries to point down (good for picking objects on a table).</summary>
            PointDown,
            /// <summary>The gripper tries to point along the target's blue (forward) axis.</summary>
            MatchTargetForward,
        }

        [Tooltip("What the gripper follows. Left empty = a test sphere is created on Play.")]
        public Transform target;
        public bool createTestTargetIfMissing = true;

        public bool followTarget = true;
        public OrientationMode orientationMode = OrientationMode.PointDown;

        [Range(0f, 1f)]
        [Tooltip("How much the orientation matters compared with the position.")]
        public float orientationWeight = 0.3f;

        [Tooltip("Distance (m) from gripper_base to the point that follows the target, along the gripper. " +
                 "Estimated value: adjust it so the yellow gizmo sits between the fingers.")]
        public float toolOffset = 0.06f;

        [Range(1, 50)]
        public int iterationsPerStep = 10;

        [Tooltip("Log a warning if the kinematic model does not match the simulated robot.")]
        public bool runSelfCheck = true;

        /// <summary>Distance (m) between the target and the tool point the IK could reach.</summary>
        public float LastPositionError { get; private set; }

        RobotArmController arm;
        readonly double[] q = new double[NiryoOneKinematics.JointCount];
        readonly double[] targetRos = new double[3];
        readonly double[] approachRos = new double[3];
        float selfCheckTime = -1f;

        void Start()
        {
            arm = GetComponent<RobotArmController>();
            if (!arm.IsReady)
            {
                enabled = false;
                return;
            }
            arm.GetCommandedJointAnglesRadians(q);

            if (target == null && createTestTargetIfMissing)
                target = CreateTestTarget();

            selfCheckTime = Time.time + 2f;
        }

        Transform CreateTestTarget()
        {
            var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.name = "IK Target (test)";
            var col = sphere.GetComponent<Collider>();
            if (col != null) Destroy(col);
            sphere.transform.localScale = Vector3.one * 0.03f;
            sphere.transform.position = ToolPointWorld(q);
            return sphere.transform;
        }

        void FixedUpdate()
        {
            if (!followTarget || target == null || arm == null) return;

            Vector3 local = arm.BaseLink.InverseTransformPoint(target.position);
            SetRos(targetRos, local);

            double[] approach = null;
            if (orientationMode == OrientationMode.PointDown)
            {
                SetRos(approachRos, arm.BaseLink.InverseTransformDirection(Vector3.down));
                approach = approachRos;
            }
            else if (orientationMode == OrientationMode.MatchTargetForward)
            {
                SetRos(approachRos, arm.BaseLink.InverseTransformDirection(target.forward));
                approach = approachRos;
            }

            LastPositionError = (float)NiryoOneKinematics.SolveIK(
                q, targetRos, approach, toolOffset, orientationWeight, iterationsPerStep);
            arm.SetJointTargetsRadians(q);
        }

        void Update()
        {
            if (runSelfCheck && selfCheckTime > 0f && Time.time >= selfCheckTime)
            {
                selfCheckTime = -1f;
                SelfCheck();
            }
        }

        /// <summary>
        /// Compares where the model says gripper_base is (from the measured joint angles)
        /// with where the physics engine actually put it.
        /// </summary>
        void SelfCheck()
        {
            if (arm.GripperBase == null) return;
            var measured = new double[NiryoOneKinematics.JointCount];
            arm.GetMeasuredJointAnglesRadians(measured);
            var p = new double[3];
            var a = new double[3];
            NiryoOneKinematics.ForwardKinematics(measured, 0.0, p, a);
            Vector3 predicted = arm.BaseLink.TransformPoint(RosToUnity(p));
            float gap = Vector3.Distance(predicted, arm.GripperBase.position);
            if (gap > 0.02f)
                Debug.LogWarning($"[ArmIKFollower] Kinematic model and simulated robot differ by {gap * 100f:F1} cm. " +
                                 "Check the robot scale or the joint directions.", this);
            else
                Debug.Log($"[ArmIKFollower] Self-check OK (gap {gap * 1000f:F1} mm).", this);
        }

        Vector3 ToolPointWorld(double[] angles)
        {
            var p = new double[3];
            var a = new double[3];
            NiryoOneKinematics.ForwardKinematics(angles, toolOffset, p, a);
            return arm.BaseLink.TransformPoint(RosToUnity(p));
        }

        // The URDF Importer maps ROS (x forward, y left, z up) to Unity as (-y, z, x).
        static void SetRos(double[] ros, Vector3 unity)
        {
            ros[0] = unity.z;
            ros[1] = -unity.x;
            ros[2] = unity.y;
        }

        static Vector3 RosToUnity(double[] ros) => new Vector3((float)-ros[1], (float)ros[2], (float)ros[0]);

        void OnDrawGizmos()
        {
            if (arm == null || !arm.IsReady) return;
            Vector3 tool = ToolPointWorld(q);
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(tool, 0.012f);
            if (target != null)
            {
                Gizmos.color = LastPositionError > 0.01f ? Color.red : Color.green;
                Gizmos.DrawLine(tool, target.position);
            }
        }
    }
}
