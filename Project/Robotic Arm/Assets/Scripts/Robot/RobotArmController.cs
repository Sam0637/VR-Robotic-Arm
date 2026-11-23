using UnityEngine;

namespace VRRoboticArm.Robot
{
    /// <summary>
    /// Drives the 6 arm joints of the imported Niryo One (ArticulationBody position drives).
    ///
    /// Put this on the robot root (the "niryo_one" object) and send angles with
    /// SetJointTargetsDegrees / SetJointTargetsRadians. The commanded angles move towards the
    /// requested ones at most <see cref="maxJointSpeed"/> deg/s, so the motion stays smooth.
    ///
    /// By default it switches off the URDF Importer keyboard "Controller", which would
    /// otherwise fight over the same drives.
    /// </summary>
    [DisallowMultipleComponent]
    public class RobotArmController : MonoBehaviour
    {
        /// <summary>Links moved by joint_1 .. joint_6 of the URDF.</summary>
        public static readonly string[] JointLinkNames =
            { "shoulder_link", "arm_link", "elbow_link", "forearm_link", "wrist_link", "hand_link" };

        [Header("Drives (same values as the URDF Importer Controller in RobotTestImport)")]
        public float stiffness = 10000f;
        public float damping = 100f;
        public float forceLimit = 1000f;

        [Tooltip("Maximum speed of each joint, in degrees per second.")]
        public float maxJointSpeed = 90f;

        [Tooltip("Disable the URDF Importer keyboard Controller (arrow keys) so it does not fight this script.")]
        public bool disableUrdfKeyboardController = true;

        ArticulationBody[] joints;
        readonly float[] requestedDeg = new float[NiryoOneKinematics.JointCount];
        readonly float[] commandedDeg = new float[NiryoOneKinematics.JointCount];

        public bool IsReady => joints != null;

        /// <summary>The base_link transform, i.e. the frame used by NiryoOneKinematics.</summary>
        public Transform BaseLink { get; private set; }

        /// <summary>The gripper_base transform (used for the self-check of the kinematic model).</summary>
        public Transform GripperBase { get; private set; }

        void Awake()
        {
            if (disableUrdfKeyboardController)
                DisableUrdfController();

            var found = new ArticulationBody[NiryoOneKinematics.JointCount];
            foreach (var body in GetComponentsInChildren<ArticulationBody>(true))
            {
                int index = System.Array.IndexOf(JointLinkNames, body.name);
                if (index >= 0) found[index] = body;
                else if (body.name == "base_link") BaseLink = body.transform;
                else if (body.name == "gripper_base") GripperBase = body.transform;
            }

            for (int i = 0; i < found.Length; i++)
            {
                if (found[i] == null)
                {
                    Debug.LogError($"[RobotArmController] Link '{JointLinkNames[i]}' not found under '{name}'. " +
                                   "Put this component on the robot root (niryo_one).", this);
                    enabled = false;
                    return;
                }
            }
            if (BaseLink == null)
            {
                Debug.LogError("[RobotArmController] 'base_link' not found under the robot root.", this);
                enabled = false;
                return;
            }

            joints = found;
            for (int i = 0; i < joints.Length; i++)
            {
                var drive = joints[i].xDrive;
                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.forceLimit = forceLimit;
                joints[i].xDrive = drive;
                requestedDeg[i] = commandedDeg[i] = drive.target;
            }
        }

        void DisableUrdfController()
        {
            // Found by name so this script does not depend on the URDF Importer assembly.
            foreach (var behaviour in GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (behaviour != null && behaviour.GetType().FullName == "Unity.Robotics.UrdfImporter.Control.Controller")
                    behaviour.enabled = false;
            }
        }

        /// <summary>Requested angles for joint_1 .. joint_6, in degrees. Clamped to the URDF limits.</summary>
        public void SetJointTargetsDegrees(float[] anglesDeg)
        {
            for (int i = 0; i < NiryoOneKinematics.JointCount; i++)
            {
                float lo = (float)(NiryoOneKinematics.LowerLimits[i] * Mathf.Rad2Deg);
                float hi = (float)(NiryoOneKinematics.UpperLimits[i] * Mathf.Rad2Deg);
                requestedDeg[i] = Mathf.Clamp(anglesDeg[i], lo, hi);
            }
        }

        /// <summary>Requested angles for joint_1 .. joint_6, in radians.</summary>
        public void SetJointTargetsRadians(double[] anglesRad)
        {
            var deg = new float[NiryoOneKinematics.JointCount];
            for (int i = 0; i < deg.Length; i++) deg[i] = (float)(anglesRad[i] * Mathf.Rad2Deg);
            SetJointTargetsDegrees(deg);
        }

        /// <summary>Measured joint angles (radians), read from the physics simulation.</summary>
        public void GetMeasuredJointAnglesRadians(double[] result)
        {
            for (int i = 0; i < NiryoOneKinematics.JointCount; i++)
                result[i] = joints[i].jointPosition.dofCount > 0 ? joints[i].jointPosition[0] : 0.0;
        }

        /// <summary>Angles currently sent to the drives (radians), after speed limiting.</summary>
        public void GetCommandedJointAnglesRadians(double[] result)
        {
            for (int i = 0; i < NiryoOneKinematics.JointCount; i++)
                result[i] = commandedDeg[i] * Mathf.Deg2Rad;
        }

        void FixedUpdate()
        {
            if (joints == null) return;
            float maxStep = maxJointSpeed * Time.fixedDeltaTime;
            for (int i = 0; i < joints.Length; i++)
            {
                commandedDeg[i] = Mathf.MoveTowards(commandedDeg[i], requestedDeg[i], maxStep);
                var drive = joints[i].xDrive;
                drive.target = commandedDeg[i];
                joints[i].xDrive = drive;
            }
        }
    }
}
