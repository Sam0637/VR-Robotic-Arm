using UnityEngine;

namespace VRRoboticArm.Robot
{
    /// <summary>
    /// Opens and closes the Niryo One gripper (prismatic joints left_gripper / right_gripper).
    ///
    /// Put this on the robot root. Call Open(), Close() or SetOpenAmount(0..1);
    /// the hand-tracking pinch gesture will call these later.
    /// The open/closed positions come from Unity's Niryo One pick-and-place tutorial
    /// (left +0.01 / -0.01 m, right mirrored) and may need tuning.
    /// </summary>
    [DisallowMultipleComponent]
    public class GripperController : MonoBehaviour
    {
        [Tooltip("Target of left_gripper when fully open (m). right_gripper gets the opposite value.")]
        public float openPosition = 0.01f;

        [Tooltip("Target of left_gripper when fully closed (m). right_gripper gets the opposite value.")]
        public float closedPosition = -0.01f;

        [Range(0f, 1f)]
        [Tooltip("1 = open, 0 = closed. Can be changed in the Inspector during Play to test.")]
        public float openAmount = 1f;

        [Tooltip("Opening/closing speed, in fraction of the full stroke per second.")]
        public float speed = 3f;

        [Header("Drives")]
        public float stiffness = 10000f;
        public float damping = 100f;
        public float forceLimit = 1000f;

        ArticulationBody left;
        ArticulationBody right;
        float currentAmount;

        public bool IsOpen => openAmount > 0.5f;

        void Awake()
        {
            foreach (var body in GetComponentsInChildren<ArticulationBody>(true))
            {
                if (body.name == "left_gripper") left = body;
                else if (body.name == "right_gripper") right = body;
            }
            if (left == null || right == null)
            {
                Debug.LogError("[GripperController] left_gripper / right_gripper not found under the robot root.", this);
                enabled = false;
                return;
            }
            Configure(left);
            Configure(right);
            currentAmount = openAmount;
        }

        void Configure(ArticulationBody body)
        {
            var drive = body.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            body.xDrive = drive;
        }

        public void Open() => openAmount = 1f;
        public void Close() => openAmount = 0f;
        public void SetOpenAmount(float amount) => openAmount = Mathf.Clamp01(amount);

        void FixedUpdate()
        {
            currentAmount = Mathf.MoveTowards(currentAmount, openAmount, speed * Time.fixedDeltaTime);
            float t = Mathf.Lerp(closedPosition, openPosition, currentAmount);
            SetTarget(left, t);
            SetTarget(right, -t);
        }

        static void SetTarget(ArticulationBody body, float target)
        {
            var drive = body.xDrive;
            drive.target = target;
            body.xDrive = drive;
        }
    }
}
