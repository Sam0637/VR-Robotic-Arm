using UnityEngine;
using UnityEngine.InputSystem;
using VRRoboticArm.Robot;
using TrackedPoseDriver = UnityEngine.InputSystem.XR.TrackedPoseDriver;

namespace VRRoboticArm.Controller
{
    [DefaultExecutionOrder(100)]
    [RequireComponent(typeof(ArmIKFollower))]
    [RequireComponent(typeof(GripperController))]
    [RequireComponent(typeof(GripperGraspAssist))]
    public class ControllerArmDriver : MonoBehaviour
    {
        public enum Hand { Right, Left }

        public Hand hand = Hand.Right;
        public Transform controller;

        [Range(0.1f, 2f)] public float motionScale = 1f;
        [Range(1f, 40f)] public float smoothing = 15f;
        [Range(0f, 0.01f)] public float deadZone = 0.002f;
        public bool followControllerRotation = false;
        public bool holdGripToMove = false;

        public float maxReach = 0.40f;
        public float minReach = 0.08f;
        public float minHeight = 0.015f;

        [Range(0.1f, 1f)] public float closeThreshold = 0.6f;
        [Range(0f, 0.9f)] public float openThreshold = 0.4f;

        public bool GripperClosed { get; private set; }
        public bool IsDriving { get; private set; }

        static readonly Vector3 ShoulderLocal = new Vector3(0f, 0.183f, 0f);

        ArmIKFollower follower;
        RobotArmController arm;
        GripperController gripper;
        Transform target;

        InputAction triggerAction;
        InputAction gripAction;
        InputAction recenterAction;
        InputAction trackedAction;

        bool anchored;
        bool rotationModeApplied;
        Vector3 controllerAnchorPos;
        Quaternion controllerAnchorRot;
        Vector3 targetAnchorPos;
        Quaternion targetAnchorRot;
        Vector3 filteredControllerPos;
        Vector3 smoothedTargetPos;

        string HandUsage => hand == Hand.Right ? "{RightHand}" : "{LeftHand}";

        void Awake()
        {
            follower = GetComponent<ArmIKFollower>();
            arm = GetComponent<RobotArmController>();
            gripper = GetComponent<GripperController>();

            target = new GameObject("Controller IK Target").transform;
            follower.target = target;
            follower.followTarget = false;

            CreateActions();
        }

        void Start()
        {
            if (arm == null || !arm.IsReady)
            {
                enabled = false;
                return;
            }

            target.position = CurrentToolPoint();
            target.rotation = Quaternion.LookRotation(Vector3.down, arm.BaseLink.forward);
            smoothedTargetPos = target.position;
            follower.followTarget = true;

            if (controller == null) controller = FindOrCreateController();
        }

        void OnEnable()
        {
            triggerAction?.Enable();
            gripAction?.Enable();
            recenterAction?.Enable();
            trackedAction?.Enable();
        }

        void OnDisable()
        {
            triggerAction?.Disable();
            gripAction?.Disable();
            recenterAction?.Disable();
            trackedAction?.Disable();
            IsDriving = false;
            anchored = false;
        }

        void OnDestroy()
        {
            triggerAction?.Dispose();
            gripAction?.Dispose();
            recenterAction?.Dispose();
            trackedAction?.Dispose();
        }

        void Update()
        {
            ApplyRotationMode();
            UpdateGripper();
            UpdateArmTarget();
        }

        void ApplyRotationMode()
        {
            if (followControllerRotation == rotationModeApplied) return;
            rotationModeApplied = followControllerRotation;
            if (followControllerRotation)
            {
                follower.orientationMode = ArmIKFollower.OrientationMode.MatchTargetForward;
            }
            else
            {
                follower.orientationMode = ArmIKFollower.OrientationMode.PointDown;
                target.rotation = Quaternion.LookRotation(Vector3.down, arm.BaseLink.forward);
            }
            anchored = false;
        }

        void UpdateGripper()
        {
            float trigger = triggerAction.ReadValue<float>();
            if (!GripperClosed && trigger >= closeThreshold)
            {
                GripperClosed = true;
                gripper.Close();
            }
            else if (GripperClosed && trigger <= openThreshold)
            {
                GripperClosed = false;
                gripper.Open();
            }
        }

        void UpdateArmTarget()
        {
            bool tracked = controller != null && trackedAction.ReadValue<float>() > 0.5f;
            bool allowed = !holdGripToMove || gripAction.ReadValue<float>() > 0.5f;

            if (!tracked || !allowed)
            {
                IsDriving = false;
                anchored = false;
                return;
            }

            if (!anchored || recenterAction.WasPressedThisFrame())
                Anchor();

            IsDriving = true;

            Vector3 raw = controller.position;
            float dist = (raw - filteredControllerPos).magnitude;
            if (dist > deadZone)
                filteredControllerPos = Vector3.MoveTowards(filteredControllerPos, raw, dist - deadZone);

            Vector3 desired = targetAnchorPos + (filteredControllerPos - controllerAnchorPos) * motionScale;
            desired = ClampToWorkspace(desired);

            float k = 1f - Mathf.Exp(-smoothing * Time.deltaTime);
            smoothedTargetPos = Vector3.Lerp(smoothedTargetPos, desired, k);
            target.position = smoothedTargetPos;

            if (followControllerRotation)
            {
                Quaternion delta = controller.rotation * Quaternion.Inverse(controllerAnchorRot);
                target.rotation = Quaternion.Slerp(target.rotation, delta * targetAnchorRot, k);
            }
        }

        void Anchor()
        {
            controllerAnchorPos = filteredControllerPos = controller.position;
            controllerAnchorRot = controller.rotation;
            targetAnchorPos = smoothedTargetPos = target.position;
            targetAnchorRot = target.rotation;
            anchored = true;
        }

        Vector3 ClampToWorkspace(Vector3 world)
        {
            Transform b = arm.BaseLink;
            Vector3 local = b.InverseTransformPoint(world);
            if (local.y < minHeight) local.y = minHeight;

            Vector3 fromShoulder = local - ShoulderLocal;
            float d = fromShoulder.magnitude;
            if (d > maxReach) fromShoulder *= maxReach / d;
            else if (d < minReach && d > 1e-5f) fromShoulder *= minReach / d;
            local = ShoulderLocal + fromShoulder;

            if (local.y < minHeight) local.y = minHeight;
            return b.TransformPoint(local);
        }

        Vector3 CurrentToolPoint()
        {
            var q = new double[NiryoOneKinematics.JointCount];
            var p = new double[3];
            var a = new double[3];
            arm.GetCommandedJointAnglesRadians(q);
            NiryoOneKinematics.ForwardKinematics(q, follower.toolOffset, p, a);
            return arm.BaseLink.TransformPoint(new Vector3((float)-p[1], (float)p[2], (float)p[0]));
        }

        void CreateActions()
        {
            string h = HandUsage;

            triggerAction = new InputAction("Gripper Trigger", InputActionType.Value, $"<XRController>{h}/trigger");
            triggerAction.AddBinding("<Keyboard>/space");

            gripAction = new InputAction("Hold To Move", InputActionType.Value, $"<XRController>{h}/grip");

            recenterAction = new InputAction("Recenter", InputActionType.Button, $"<XRController>{h}/primaryButton");
            recenterAction.AddBinding("<Keyboard>/r");

            trackedAction = new InputAction("Is Tracked", InputActionType.Value, $"<XRController>{h}/isTracked");
        }

        Transform FindOrCreateController()
        {
            string side = hand == Hand.Right ? "Right" : "Left";
            foreach (string n in new[] { $"{side} Controller", $"{side}Hand Controller", $"{side} Hand Controller" })
            {
                var found = GameObject.Find(n);
                if (found != null) return found.transform;
            }

            Camera cam = Camera.main;
            Transform space = cam != null ? cam.transform.parent : null;

            var go = new GameObject($"{side} Controller");
            go.SetActive(false);
            go.transform.SetParent(space, false);
            var tpd = go.AddComponent<TrackedPoseDriver>();
            tpd.trackingType = TrackedPoseDriver.TrackingType.RotationAndPosition;
            tpd.positionInput = new InputActionProperty(
                new InputAction("Position", InputActionType.Value, $"<XRController>{HandUsage}/devicePosition", expectedControlType: "Vector3"));
            tpd.rotationInput = new InputActionProperty(
                new InputAction("Rotation", InputActionType.Value, $"<XRController>{HandUsage}/deviceRotation", expectedControlType: "Quaternion"));
            go.SetActive(true);
            return go.transform;
        }
    }
}
