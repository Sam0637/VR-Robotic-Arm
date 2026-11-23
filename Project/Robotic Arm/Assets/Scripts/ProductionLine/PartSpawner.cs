using System.Collections.Generic;
using UnityEngine;

namespace VRRoboticArm.ProductionLine
{
    /// <summary>
    /// Drops a part of random color on the conveyor at regular intervals.
    /// Waits while the previous part is still too close, so parts never pile up.
    /// </summary>
    public class PartSpawner : MonoBehaviour
    {
        [Tooltip("Optional prefab (needs a Rigidbody). Empty = a small cube is created.")]
        public GameObject partPrefab;

        [Tooltip("Seconds between two parts.")]
        public float interval = 4f;

        [Tooltip("Edge size of the generated cube (m). The gripper opening is small: keep it around 3 cm.")]
        public float partSize = 0.03f;

        public float partMass = 0.05f;

        [Tooltip("A new part only appears when the last one is at least this far (m).")]
        public float minSpacing = 0.12f;

        [Tooltip("Maximum number of parts present at the same time.")]
        public int maxParts = 6;

        public bool spawning = true;

        readonly List<GameObject> alive = new List<GameObject>();
        float timer;

        void Update()
        {
            alive.RemoveAll(p => p == null);
            if (!spawning) return;

            timer += Time.deltaTime;
            if (timer < interval || alive.Count >= maxParts) return;

            GameObject last = alive.Count > 0 ? alive[alive.Count - 1] : null;
            if (last != null && Vector3.Distance(last.transform.position, transform.position) < minSpacing) return;

            timer = 0f;
            alive.Add(Spawn((PartColor)Random.Range(0, 3)));
        }

        GameObject Spawn(PartColor color)
        {
            GameObject part;
            if (partPrefab != null)
            {
                part = Instantiate(partPrefab, transform.position, transform.rotation);
            }
            else
            {
                part = GameObject.CreatePrimitive(PrimitiveType.Cube);
                part.transform.SetPositionAndRotation(transform.position, transform.rotation);
                part.transform.localScale = Vector3.one * partSize;
            }
            part.name = $"Part ({color})";

            var rb = part.GetComponent<Rigidbody>();
            if (rb == null) rb = part.AddComponent<Rigidbody>();
            rb.mass = partMass;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

            var info = part.GetComponent<ProductionPart>();
            if (info == null) info = part.AddComponent<ProductionPart>();
            info.color = color;

            var renderer = part.GetComponent<Renderer>();
            if (renderer != null) renderer.material.color = ProductionPart.ToUnityColor(color);
            return part;
        }
    }
}
