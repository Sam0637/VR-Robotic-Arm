using UnityEngine;

namespace VRRoboticArm.ProductionLine
{
    /// <summary>
    /// Counts correctly sorted, wrongly sorted and missed parts,
    /// and shows them on a 3D text (visible in VR, unlike screen UI).
    /// </summary>
    public class ProductionScore : MonoBehaviour
    {
        public TextMesh display;

        public int Correct { get; private set; }
        public int Wrong { get; private set; }
        public int Missed { get; private set; }

        void Start() => Refresh();

        public void Report(bool correct)
        {
            if (correct) Correct++;
            else Wrong++;
            Refresh();
        }

        public void ReportMissed()
        {
            Missed++;
            Refresh();
        }

        public void ResetScore()
        {
            Correct = Wrong = Missed = 0;
            Refresh();
        }

        void Refresh()
        {
            if (display != null)
                display.text = $"Bien triées : {Correct}\nErreurs : {Wrong}\nManquées : {Missed}";
        }
    }
}
