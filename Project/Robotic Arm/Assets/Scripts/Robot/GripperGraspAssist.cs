using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace VRRoboticArm.Robot
{
    [DefaultExecutionOrder(200)]
    [RequireComponent(typeof(RobotArmController))]
    [RequireComponent(typeof(GripperController))]
    public class GripperGraspAssist : MonoBehaviour
    {
        public float grabRadius = 0.025f;
        public float grabWindow = 0.5f;
        public float maxMass = 1f;
        public float toolOffset = 0.06f;
        public float releaseNoCollisionTime = 0.25f;
        public bool keepVelocityOnRelease = true;

        public Rigidbody Held { get; private set; }

        public event System.Action<Rigidbody> Grasped;
        public event System.Action<Rigidbody> Released;

        RobotArmController arm;
        GripperController gripper;
        ArticulationBody leftFinger;
        Collider[] robotColliders;

        bool wasOpen = true;
        float closeTime = -1f;

        Vector3 heldLocalPos;
        Quaternion heldLocalRot;
        bool heldWasKinematic;
        bool heldUsedGravity;
        RigidbodyInterpolation heldInterpolation;
        Vector3 lastHeldPos;
        Vector3 heldVelocity;

        readonly Collider[] overlap = new Collider[16];
        readonly double[] q = new double[NiryoOneKinematics.JointCount];
        readonly List<Collider> heldColliders = new List<Collider>();

        void Awake()
        {
            arm = GetComponent<RobotArmController>();
            gripper = GetComponent<GripperController>();
            foreach (var body in GetComponentsInChildren<ArticulationBody>(true))
                if (body.name == "left_gripper") leftFinger = body;
            robotColliders = GetComponentsInChildren<Collider>(true);

            var follower = GetComponent<ArmIKFollower>();
            if (follower != null) toolOffset = follower.toolOffset;
        }

        void FixedUpdate()
        {
            if (arm == null || !arm.IsReady || arm.GripperBase == null) return;

            bool isOpen = gripper.IsOpen;
            if (wasOpen && !isOpen) closeTime = Time.time;
            if (!wasOpen && isOpen) Release();
            wasOpen = isOpen;

            if (Held == null && !isOpen && closeTime >= 0f && Time.time - closeTime <= grabWindow)
                TryGrab();

            if (Held != null) FollowGripper();
        }

        void TryGrab()
        {
            Vector3 tool = ToolPoint();
            int n = Physics.OverlapSphereNonAlloc(tool, grabRadius, overlap, ~0, QueryTriggerInteraction.Ignore);

            Rigidbody best = null;
            float bestDist = float.MaxValue;
            for (int i = 0; i < n; i++)
            {
                var rb = overlap[i].attachedRigidbody;
                if (rb == null || rb.isKinematic || rb.mass > maxMass) continue;
                if (rb.GetComponentInParent<ArticulationBody>() != null) continue;
                float d = Vector3.Distance(overlap[i].ClosestPoint(tool), tool);
                if (d < bestDist) { bestDist = d; best = rb; }
            }
            if (best != null) Grab(best);
        }

        void Grab(Rigidbody rb)
        {
            Held = rb;
            Transform g = arm.GripperBase;
            heldLocalPos = g.InverseTransformPoint(rb.position);
            heldLocalRot = Quaternion.Inverse(g.rotation) * rb.rotation;

            heldWasKinematic = rb.isKinematic;
            heldUsedGravity = rb.useGravity;
            heldInterpolation = rb.interpolation;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.isKinematic = true;
            rb.useGravity = false;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            lastHeldPos = rb.position;
            heldVelocity = Vector3.zero;

            SetCollisionWithRobot(rb, false);

            if (leftFinger != null && leftFinger.jointPosition.dofCount > 0)
            {
                float pos = leftFinger.jointPosition[0];
                gripper.SetOpenAmount(Mathf.InverseLerp(gripper.closedPosition, gripper.openPosition, pos));
            }
            if (gripper.IsOpen) gripper.SetOpenAmount(0.49f);
            wasOpen = false;

            Grasped?.Invoke(rb);
        }

        void FollowGripper()
        {
            if (Held == null) return;
            Transform g = arm.GripperBase;
            Vector3 p = g.TransformPoint(heldLocalPos);
            Held.MovePosition(p);
            Held.MoveRotation(g.rotation * heldLocalRot);

            heldVelocity = (p - lastHeldPos) / Time.fixedDeltaTime;
            lastHeldPos = p;
        }

        void Release()
        {
            closeTime = -1f;
            Rigidbody rb = Held;
            Held = null;
            if (rb == null) return;

            rb.isKinematic = heldWasKinematic;
            rb.useGravity = heldUsedGravity;
            rb.interpolation = heldInterpolation;
            if (!rb.isKinematic)
            {
                rb.linearVelocity = keepVelocityOnRelease ? heldVelocity : Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }

            if (isActiveAndEnabled) StartCoroutine(RestoreCollisionsLater(rb));
            else SetCollisionWithRobot(rb, true);

            Released?.Invoke(rb);
        }

        IEnumerator RestoreCollisionsLater(Rigidbody rb)
        {
            yield return new WaitForSeconds(releaseNoCollisionTime);
            if (rb != null && rb != Held) SetCollisionWithRobot(rb, true);
        }

        void OnDisable()
        {
            if (Held != null) Release();
        }

        void SetCollisionWithRobot(Rigidbody rb, bool collide)
        {
            rb.GetComponentsInChildren(heldColliders);
            foreach (var c in heldColliders)
                foreach (var r in robotColliders)
                    if (c != null && r != null) Physics.IgnoreCollision(c, r, !collide);
        }

        Vector3 ToolPoint()
        {
            var p = new double[3];
            var a = new double[3];
            arm.GetMeasuredJointAnglesRadians(q);
            NiryoOneKinematics.ForwardKinematics(q, toolOffset, p, a);
            return arm.BaseLink.TransformPoint(new Vector3((float)-p[1], (float)p[2], (float)p[0]));
        }
    }
}
