using System.Collections.Generic;
using UnityEngine;

namespace VRRoboticArm.ProductionLine
{
    /// <summary>
    /// Invisible zone in front of the robot: the belt stops while a part is inside,
    /// and starts again once the part has been picked up.
    /// Needs a Collider with "Is Trigger" checked.
    /// </summary>
    [RequireComponent(typeof(Collider))]
    public class ConveyorStopZone : MonoBehaviour
    {
        public ConveyorBelt belt;

        readonly HashSet<ProductionPart> inside = new HashSet<ProductionPart>();

        void Awake() => GetComponent<Collider>().isTrigger = true;

        void OnTriggerEnter(Collider other)
        {
            var part = other.GetComponentInParent<ProductionPart>();
            if (part != null) inside.Add(part);
        }

        void OnTriggerExit(Collider other)
        {
            var part = other.GetComponentInParent<ProductionPart>();
            if (part != null) inside.Remove(part);
        }

        void FixedUpdate()
        {
            inside.RemoveWhere(p => p == null);
            if (belt != null) belt.running = inside.Count == 0;
        }
    }
}
