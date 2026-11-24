using UnityEngine;

namespace VRRoboticArm.ProductionLine
{
    /// <summary>
    /// Builds a small production line around the Niryo One when Play starts:
    /// conveyor, part spawner, stop zone in front of the robot, 3 sorting bins and score board.
    /// Optional (off by default, for mixed reality): a pedestal under the robot and a simple factory hall.
    ///
    /// Usage: create an empty GameObject, add this component, press Play.
    /// Positions are given in the robot frame (x = in front of the robot, y = robot's left,
    /// z = up, in metres from the robot base), so the line always sits within the arm's reach.
    /// The layout is previewed in the Scene view (yellow boxes) when this object is selected.
    /// </summary>
    public class ProductionLineBuilder : MonoBehaviour
    {
        [Tooltip("The robot's base_link (the object with the ArticulationBody). Empty = found automatically.")]
        public Transform robotBase;

        public bool buildOnStart = true;

        [Header("Pedestal (robot and conveyor stand on it)")]
        [Tooltip("Off for mixed reality: the real table carries the robot and the conveyor.")]
        public bool createPedestal = false;

        [Tooltip("Height of the robot base above the floor (0.63 m in niryo_one.urdf).")]
        public float tableHeight = 0.63f;

        [Header("Conveyor")]
        [Tooltip("Distance from the robot base to the belt centre line (m). The arm reaches about 0.4 m.")]
        public float conveyorDistance = 0.22f;
        public float conveyorLength = 1.3f;
        public float conveyorWidth = 0.12f;
        public float conveyorSpeed = 0.06f;
        public float spawnInterval = 4f;

        [Header("Factory hall")]
        [Tooltip("Off for mixed reality: the real room is seen through the headset cameras.")]
        public bool createFactoryHall = false;
        public float hallSize = 16f;
        public float hallHeight = 6f;

        static readonly Color Steel = new Color(0.55f, 0.57f, 0.6f);
        static readonly Color DarkSteel = new Color(0.25f, 0.26f, 0.28f);
        static readonly Color Belt = new Color(0.08f, 0.08f, 0.09f);
        static readonly Color Concrete = new Color(0.42f, 0.42f, 0.4f);
        static readonly Color Wall = new Color(0.62f, 0.6f, 0.56f);
        static readonly Color SafetyYellow = new Color(0.95f, 0.75f, 0.05f);

        Transform root;

        void Start()
        {
            if (buildOnStart) Build();
        }

        /// <summary>Creates the whole line. Can also be called from another script.</summary>
        public void Build()
        {
            if (robotBase == null) robotBase = FindBase();
            if (robotBase == null)
            {
                Debug.LogError("[ProductionLineBuilder] No 'base_link' found. Assign the robot base in the Inspector.", this);
                return;
            }

            // The robot base must never move, otherwise the line is no longer in reach.
            var baseBody = robotBase.GetComponent<ArticulationBody>();
            if (baseBody != null) baseBody.immovable = true;

            // Created at the scene root so the builder object's own transform has no effect.
            root = new GameObject("Production Line (generated)").transform;

            if (createPedestal) BuildPedestal();
            var score = BuildScoreBoard();
            BuildConveyor(score);
            BuildBins(score);
            if (createFactoryHall) BuildHall();
        }

        // ---------------------------------------------------------------- parts of the line

        /// <summary>
        /// Solid block from the floor up to the robot base, carrying the robot, the conveyor and the bins.
        /// Its top is 2 mm below the robot base so it never touches the robot's colliders.
        /// </summary>
        void BuildPedestal()
        {
            var pedestal = Group("Pedestal");
            float x0 = -0.32f, x1 = conveyorDistance + conveyorWidth / 2f + 0.15f;
            float halfY = conveyorLength / 2f + 0.15f;
            float h = tableHeight - 0.002f;

            Box("Block", pedestal, new Vector3((x0 + x1) / 2f, 0f, -0.002f - h / 2f), new Vector3(x1 - x0, 2f * halfY, h), Steel);
            // Yellow safety band along the top edge facing the user.
            Box("Edge Band", pedestal, new Vector3(x1 + 0.001f, 0f, -0.03f), new Vector3(0.004f, 2f * halfY, 0.04f), SafetyYellow, false);
        }

        void BuildConveyor(ProductionScore score)
        {
            var conveyor = Group("Conveyor");
            float d = conveyorDistance, half = conveyorLength / 2f;

            // Belt: kinematic rigidbody moved by ConveyorBelt.
            var belt = Box("Belt", conveyor, new Vector3(d, 0f, 0.02f), new Vector3(conveyorWidth, conveyorLength, 0.04f), Belt);
            belt.AddComponent<Rigidbody>();
            var beltScript = belt.AddComponent<ConveyorBelt>();
            beltScript.speed = conveyorSpeed;
            beltScript.localDirection = Vector3.right; // robot's right = towards -y in the robot frame

            // Low side rails keep the parts on the belt.
            foreach (float side in new[] { -1f, 1f })
                Box("Rail", conveyor, new Vector3(d + side * (conveyorWidth / 2f + 0.005f), 0f, 0.05f),
                    new Vector3(0.01f, conveyorLength, 0.02f), Steel);

            // End rollers (visual only).
            foreach (float y in new[] { -half, half })
            {
                var roller = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                roller.name = "Roller";
                Destroy(roller.GetComponent<Collider>());
                roller.transform.SetParent(conveyor, false);
                roller.transform.position = P(d, y, 0.02f);
                roller.transform.rotation = robotBase.rotation * Quaternion.Euler(90f, 0f, 0f);
                roller.transform.localScale = new Vector3(0.045f, conveyorWidth / 2f, 0.045f);
                Paint(roller, DarkSteel);
            }

            // Spawner above the start of the belt (robot's left).
            var spawner = new GameObject("Part Spawner");
            spawner.transform.SetParent(conveyor, false);
            spawner.transform.SetPositionAndRotation(P(d, half - 0.07f, 0.08f), robotBase.rotation);
            spawner.AddComponent<PartSpawner>().interval = spawnInterval;
            Box("Hopper", conveyor, new Vector3(d, half - 0.07f, 0.14f), new Vector3(0.09f, 0.09f, 0.06f), SafetyYellow, false);

            // Stop zone in front of the robot: a part stops at about y = 0.
            var stop = Zone("Stop Zone", conveyor, new Vector3(d, -0.04f, 0.06f), new Vector3(conveyorWidth, 0.05f, 0.06f));
            stop.AddComponent<ConveyorStopZone>().belt = beltScript;

            // End of the belt: parts falling there were missed.
            var missed = Zone("Missed Zone", conveyor, new Vector3(d, -half - 0.05f, -0.1f), new Vector3(0.3f, 0.16f, 0.35f));
            missed.AddComponent<MissedPartZone>().score = score;
        }

        void BuildBins(ProductionScore score)
        {
            var bins = Group("Bins");
            // Positions checked with the IK: all reachable with the gripper pointing down.
            MakeBin(bins, "Bin Red", new Vector2(0.02f, 0.27f), PartColor.Red, score);
            MakeBin(bins, "Bin Green", new Vector2(0.02f, -0.27f), PartColor.Green, score);
            MakeBin(bins, "Bin Blue", new Vector2(-0.18f, 0.12f), PartColor.Blue, score);
        }

        void MakeBin(Transform parent, string name, Vector2 xy, PartColor color, ProductionScore score)
        {
            var bin = Group(name, parent);
            Color c = ProductionPart.ToUnityColor(color);
            const float size = 0.12f, wall = 0.008f, height = 0.06f;
            float x = xy.x, y = xy.y;

            Box("Bottom", bin, new Vector3(x, y, wall / 2f), new Vector3(size, size, wall), c);
            Box("Wall", bin, new Vector3(x + size / 2f, y, height / 2f), new Vector3(wall, size, height), c);
            Box("Wall", bin, new Vector3(x - size / 2f, y, height / 2f), new Vector3(wall, size, height), c);
            Box("Wall", bin, new Vector3(x, y + size / 2f, height / 2f), new Vector3(size, wall, height), c);
            Box("Wall", bin, new Vector3(x, y - size / 2f, height / 2f), new Vector3(size, wall, height), c);

            var trigger = Zone("Detector", bin, new Vector3(x, y, height / 2f), new Vector3(size - 2 * wall, size - 2 * wall, height - wall));
            var sorter = trigger.AddComponent<SortingBin>();
            sorter.acceptedColor = color;
            sorter.score = score;
        }

        ProductionScore BuildScoreBoard()
        {
            var board = Group("Score Board");
            // Behind the robot, facing the user (who stands in front of the conveyor).
            Box("Post", board, new Vector3(-0.30f, 0f, 0.15f), new Vector3(0.03f, 0.03f, 0.3f), DarkSteel);
            Box("Panel", board, new Vector3(-0.30f, 0f, 0.45f), new Vector3(0.02f, 0.5f, 0.3f), DarkSteel);

            var textGo = new GameObject("Score Text");
            textGo.transform.SetParent(board, false);
            textGo.transform.position = P(-0.285f, 0f, 0.45f);
            textGo.transform.rotation = robotBase.rotation * Quaternion.LookRotation(Vector3.back);

            var text = textGo.AddComponent<TextMesh>();
            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            text.font = font;
            textGo.GetComponent<MeshRenderer>().sharedMaterial = font.material;
            text.fontSize = 80;
            text.characterSize = 0.006f;
            text.anchor = TextAnchor.MiddleCenter;
            text.alignment = TextAlignment.Left;
            text.color = Color.white;

            var score = board.gameObject.AddComponent<ProductionScore>();
            score.display = text;
            return score;
        }

        void BuildHall()
        {
            // Simple placeholder hall, centred on the robot, until a factory asset pack is chosen.
            var hall = Group("Factory Hall");
            Vector3 floorCentre = robotBase.position - Vector3.up * tableHeight;
            float s = hallSize, h = hallHeight;

            // Top 1 mm below the floor level, so an existing ground plane does not flicker with it.
            WorldBox("Floor", hall, floorCentre - Vector3.up * 0.051f, new Vector3(s, 0.1f, s), Concrete);
            WorldBox("Wall N", hall, floorCentre + new Vector3(0, h / 2f, s / 2f), new Vector3(s, h, 0.2f), Wall);
            WorldBox("Wall S", hall, floorCentre + new Vector3(0, h / 2f, -s / 2f), new Vector3(s, h, 0.2f), Wall);
            WorldBox("Wall E", hall, floorCentre + new Vector3(s / 2f, h / 2f, 0), new Vector3(0.2f, h, s), Wall);
            WorldBox("Wall W", hall, floorCentre + new Vector3(-s / 2f, h / 2f, 0), new Vector3(0.2f, h, s), Wall);

            // Steel pillars.
            for (float px = -s / 2f + 4f; px < s / 2f - 1f; px += 4f)
                for (float pz = -s / 2f + 4f; pz < s / 2f - 1f; pz += 4f)
                    if (Mathf.Abs(px) > 1.5f || Mathf.Abs(pz) > 1.5f)
                        WorldBox("Pillar", hall, floorCentre + new Vector3(px, h / 2f, pz), new Vector3(0.3f, h, 0.3f), DarkSteel);

            // Yellow safety lines around the robot cell.
            const float cell = 1.2f, line = 0.06f;
            WorldBox("Safety Line", hall, floorCentre + new Vector3(0, 0.002f, cell), new Vector3(2 * cell, 0.004f, line), SafetyYellow, false);
            WorldBox("Safety Line", hall, floorCentre + new Vector3(0, 0.002f, -cell), new Vector3(2 * cell, 0.004f, line), SafetyYellow, false);
            WorldBox("Safety Line", hall, floorCentre + new Vector3(cell, 0.002f, 0), new Vector3(line, 0.004f, 2 * cell), SafetyYellow, false);
            WorldBox("Safety Line", hall, floorCentre + new Vector3(-cell, 0.002f, 0), new Vector3(line, 0.004f, 2 * cell), SafetyYellow, false);
        }

        // ---------------------------------------------------------------- helpers

        /// <summary>Robot frame (x front, y left, z up) to world position.</summary>
        Vector3 P(float x, float y, float z) => robotBase.TransformPoint(new Vector3(-y, z, x));

        /// <summary>Box with its centre and size given in the robot frame.</summary>
        GameObject Box(string name, Transform parent, Vector3 centre, Vector3 size, Color color, bool collider = true)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = name;
            if (!collider) Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(parent, false);
            go.transform.SetPositionAndRotation(P(centre.x, centre.y, centre.z), robotBase.rotation);
            go.transform.localScale = new Vector3(size.y, size.z, size.x);
            Paint(go, color);
            return go;
        }

        /// <summary>Box with its centre and size given in world axes.</summary>
        GameObject WorldBox(string name, Transform parent, Vector3 centre, Vector3 size, Color color, bool collider = true)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = name;
            if (!collider) Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(parent, false);
            go.transform.position = centre;
            go.transform.localScale = size;
            Paint(go, color);
            return go;
        }

        /// <summary>Invisible trigger volume, centre and size in the robot frame.</summary>
        GameObject Zone(string name, Transform parent, Vector3 centre, Vector3 size)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.transform.SetPositionAndRotation(P(centre.x, centre.y, centre.z), robotBase.rotation);
            var box = go.AddComponent<BoxCollider>();
            box.size = new Vector3(size.y, size.z, size.x);
            box.isTrigger = true;
            return go;
        }

        Transform Group(string name, Transform parent = null)
        {
            var t = new GameObject(name).transform;
            t.SetParent(parent != null ? parent : root, false);
            return t;
        }

        static void Paint(GameObject go, Color color)
        {
            var r = go.GetComponent<Renderer>();
            if (r != null) r.material.color = color;
        }

        /// <summary>
        /// The robot's base_link is the ArticulationBody with that name. Searching by GameObject name
        /// is not enough: the imported meshes contain other objects also called "base_link".
        /// </summary>
        static Transform FindBase()
        {
            foreach (var body in FindObjectsByType<ArticulationBody>(FindObjectsSortMode.None))
                if (body.name == "base_link") return body.transform;
            return null;
        }

        void OnDrawGizmosSelected()
        {
            Transform b = robotBase != null ? robotBase : FindBase();
            if (b == null) return;
            Gizmos.color = Color.yellow;
            Gizmos.matrix = Matrix4x4.TRS(b.position, b.rotation, Vector3.one);
            // Belt, then the three bins (robot frame converted to Unity local axes: (-y, z, x)).
            Gizmos.DrawWireCube(new Vector3(0f, 0.02f, conveyorDistance), new Vector3(conveyorLength, 0.04f, conveyorWidth));
            Gizmos.DrawWireCube(new Vector3(-0.27f, 0.03f, 0.02f), new Vector3(0.12f, 0.06f, 0.12f));
            Gizmos.DrawWireCube(new Vector3(0.27f, 0.03f, 0.02f), new Vector3(0.12f, 0.06f, 0.12f));
            Gizmos.DrawWireCube(new Vector3(-0.12f, 0.03f, -0.18f), new Vector3(0.12f, 0.06f, 0.12f));
        }
    }
}
