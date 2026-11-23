using UnityEngine;

namespace VRRoboticArm.ProductionLine
{
    public enum PartColor { Red, Green, Blue }

    /// <summary>A part produced by the line. Knows its color and whether it was already counted.</summary>
    [RequireComponent(typeof(Rigidbody))]
    public class ProductionPart : MonoBehaviour
    {
        public PartColor color;

        /// <summary>True once a bin or the end of the line has counted this part.</summary>
        public bool Counted { get; set; }

        [Tooltip("The part is destroyed if it falls this far below its spawn height (m).")]
        public float maxDrop = 2f;

        float spawnHeight;

        void Start() => spawnHeight = transform.position.y;

        void Update()
        {
            if (transform.position.y < spawnHeight - maxDrop)
                Destroy(gameObject);
        }

        public static Color ToUnityColor(PartColor c)
        {
            switch (c)
            {
                case PartColor.Red: return new Color(0.85f, 0.15f, 0.12f);
                case PartColor.Green: return new Color(0.15f, 0.7f, 0.25f);
                default: return new Color(0.15f, 0.35f, 0.9f);
            }
        }
    }
}
