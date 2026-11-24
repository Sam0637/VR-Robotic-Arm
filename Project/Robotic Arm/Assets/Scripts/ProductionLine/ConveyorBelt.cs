using UnityEngine;

namespace VRRoboticArm.ProductionLine
{
    /// <summary>
    /// Conveyor belt that carries the rigidbodies lying on it.
    ///
    /// Trick: the belt is a kinematic Rigidbody. Each physics step it is moved back, then
    /// "moved" forward with MovePosition. It stays in place, but the physics engine sees a
    /// moving surface and drags the objects on it by friction.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class ConveyorBelt : MonoBehaviour
    {
        [Tooltip("Belt speed in m/s.")]
        public float speed = 0.06f;

        [Tooltip("Direction of travel, in the belt's local axes.")]
        public Vector3 localDirection = Vector3.right;

        [Tooltip("Uncheck to stop the belt.")]
        public bool running = true;

        Rigidbody rb;

        void Awake()
        {
            rb = GetComponent<Rigidbody>();
            rb.isKinematic = true;
            rb.useGravity = false;
        }

        void FixedUpdate()
        {
            if (!running || speed == 0f) return;
            Vector3 move = transform.TransformDirection(localDirection.normalized) * speed * Time.fixedDeltaTime;
            rb.position -= move;
            rb.MovePosition(rb.position + move);
        }
    }
}
