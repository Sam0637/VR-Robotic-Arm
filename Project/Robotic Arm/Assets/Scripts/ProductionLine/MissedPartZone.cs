using UnityEngine;

namespace VRRoboticArm.ProductionLine
{
    /// <summary>
    /// Zone at the end of the conveyor: a part reaching it was not picked up.
    /// It counts as missed and is removed. Needs a Collider with "Is Trigger" checked.
    /// </summary>
    [RequireComponent(typeof(Collider))]
    public class MissedPartZone : MonoBehaviour
    {
        public ProductionScore score;

        void Awake() => GetComponent<Collider>().isTrigger = true;

        void OnTriggerEnter(Collider other)
        {
            var part = other.GetComponentInParent<ProductionPart>();
            if (part == null || part.Counted) return;
            part.Counted = true;
            if (score != null) score.ReportMissed();
            Destroy(part.gameObject);
        }
    }
}
