using System.Collections.Generic;
using UnityEngine;

namespace VRRoboticArm.ProductionLine
{
    /// <summary>
    /// Bin that accepts one color. A part counts once it has stayed inside for
    /// <see cref="settleTime"/> seconds (so a part only passing through does not count).
    /// Needs a Collider with "Is Trigger" checked, covering the inside of the bin.
    /// </summary>
    [RequireComponent(typeof(Collider))]
    public class SortingBin : MonoBehaviour
    {
        public PartColor acceptedColor;
        public ProductionScore score;

        public float settleTime = 0.5f;

        [Tooltip("Counted parts are removed after this delay (s).")]
        public float removeDelay = 1.5f;

        readonly Dictionary<ProductionPart, float> timers = new Dictionary<ProductionPart, float>();

        void Awake() => GetComponent<Collider>().isTrigger = true;

        void OnTriggerStay(Collider other)
        {
            var part = other.GetComponentInParent<ProductionPart>();
            if (part == null || part.Counted) return;

            var body = part.GetComponent<Rigidbody>();
            if (body != null && body.isKinematic) { timers.Remove(part); return; }

            timers.TryGetValue(part, out float t);
            t += Time.fixedDeltaTime;
            timers[part] = t;
            if (t < settleTime) return;

            part.Counted = true;
            timers.Remove(part);
            if (score != null) score.Report(part.color == acceptedColor);
            Destroy(part.gameObject, removeDelay);
        }

        void OnTriggerExit(Collider other)
        {
            var part = other.GetComponentInParent<ProductionPart>();
            if (part != null) timers.Remove(part);
        }
    }
}
