using System;

namespace VRRoboticArm.Robot
{
    /// <summary>
    /// Forward and inverse kinematics of the Niryo One arm (6 revolute joints).
    ///
    /// All math is done in the ROS frame of <c>base_link</c> (right-handed: x forward, y left, z up),
    /// with the joint origins copied from Assets/URDF/niryo_one/niryo_one.urdf.
    /// Joint angles are in radians here; the ArticulationBody drives use degrees.
    ///
    /// This class has no Unity dependency so it can be tested outside the editor.
    /// </summary>
    public static class NiryoOneKinematics
    {
        public const int JointCount = 6;

        // URDF <limit lower/upper> of joint_1 .. joint_6 (radians).
        public static readonly double[] LowerLimits = { -3.05433, -1.91986, -1.397485, -3.05433, -1.74533, -2.57436 };
        public static readonly double[] UpperLimits = { 3.05433, 0.640187, 1.5707963268, 3.05433, 1.91986, 2.57436 };

        // URDF <origin xyz rpy> of joint_1 .. joint_6. Every joint turns around its local z axis.
        static readonly double[,] JointXyz =
        {
            { 0, 0, 0.103 },
            { 0, 0, 0.08 },
            { 0.21, 0, 0 },
            { 0.0415, 0.03, 0 },
            { 0, 0, 0.18 },
            { 0.0164, -0.0055, 0 },
        };
        static readonly double[,] JointRpy =
        {
            { 0, 0, 0 },
            { 1.5707963268, -1.5707963268, 0 },
            { 0, 0, -1.5707963268 },
            { 0, 1.5707963268, 0 },
            { 0, -1.5707963268, 0 },
            { 0, 1.5707963268, 0 },
        };

        // Fixed joints hand_link -> tool_link -> gripper_base.
        static readonly double[,] HandToGripperBase =
            Mul(Pose(0, 0, 0.0073, -1.5707963268, -1.5707963268, 0),
                Pose(0.0246, 0, -0.01475, 1.5707, 0, -1.5707));

        static readonly double[][,] JointOrigins = BuildJointOrigins();

        static double[][,] BuildJointOrigins()
        {
            var origins = new double[JointCount][,];
            for (int i = 0; i < JointCount; i++)
                origins[i] = Pose(JointXyz[i, 0], JointXyz[i, 1], JointXyz[i, 2], JointRpy[i, 0], JointRpy[i, 1], JointRpy[i, 2]);
            return origins;
        }

        /// <summary>
        /// Pose of the tool point in the base_link ROS frame.
        /// The approach axis is the direction the gripper points to (-z of gripper_base).
        /// The tool point is <paramref name="toolOffset"/> metres from gripper_base along that axis.
        /// </summary>
        public static void ForwardKinematics(double[] q, double toolOffset, double[] position, double[] approach)
        {
            double[,] m = Identity();
            for (int i = 0; i < JointCount; i++)
                m = Mul(Mul(m, JointOrigins[i]), RotZ(q[i]));
            m = Mul(m, HandToGripperBase);

            for (int r = 0; r < 3; r++)
            {
                approach[r] = -m[r, 2];
                position[r] = m[r, 3] + approach[r] * toolOffset;
            }
        }

        /// <summary>
        /// Damped least squares IK, warm-started from <paramref name="q"/> (modified in place, radians).
        /// If <paramref name="targetApproach"/> is null, only the position is solved.
        /// Otherwise the gripper also tries to point along targetApproach (unit vector),
        /// with <paramref name="orientationWeight"/> trading position vs. orientation accuracy.
        /// Returns the remaining position error in metres.
        /// </summary>
        public static double SolveIK(double[] q, double[] targetPosition, double[] targetApproach,
                                     double toolOffset, double orientationWeight = 0.3,
                                     int iterations = 10, double damping = 0.05, double maxStepRad = 0.2)
        {
            int m = targetApproach == null ? 3 : 6;
            var e = new double[m];
            var ePlus = new double[m];
            var jac = new double[m, JointCount];
            var qPlus = new double[JointCount];
            const double h = 1e-5;

            for (int it = 0; it < iterations; it++)
            {
                Error(q, targetPosition, targetApproach, toolOffset, orientationWeight, e);
                if (Norm(e) < 1e-4) break;

                // Numerical Jacobian of the tool pose: J = (e(q) - e(q + h)) / h, because e = target - f(q).
                for (int k = 0; k < JointCount; k++)
                {
                    Array.Copy(q, qPlus, JointCount);
                    qPlus[k] += h;
                    Error(qPlus, targetPosition, targetApproach, toolOffset, orientationWeight, ePlus);
                    for (int r = 0; r < m; r++) jac[r, k] = (e[r] - ePlus[r]) / h;
                }

                // dq = J^T (J J^T + lambda^2 I)^-1 e
                var a = new double[m, m];
                for (int r = 0; r < m; r++)
                    for (int c = 0; c < m; c++)
                    {
                        double s = 0;
                        for (int k = 0; k < JointCount; k++) s += jac[r, k] * jac[c, k];
                        a[r, c] = s + (r == c ? damping * damping : 0);
                    }
                double[] y = SolveLinear(a, e);
                if (y == null) break;

                double maxAbs = 0;
                var dq = new double[JointCount];
                for (int k = 0; k < JointCount; k++)
                {
                    double s = 0;
                    for (int r = 0; r < m; r++) s += jac[r, k] * y[r];
                    dq[k] = s;
                    maxAbs = Math.Max(maxAbs, Math.Abs(s));
                }
                double scale = maxAbs > maxStepRad ? maxStepRad / maxAbs : 1.0;
                for (int k = 0; k < JointCount; k++)
                    q[k] = Clamp(q[k] + dq[k] * scale, LowerLimits[k], UpperLimits[k]);
            }

            Error(q, targetPosition, null, toolOffset, 0, e);
            return Math.Sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        }

        static void Error(double[] q, double[] tp, double[] ta, double toolOffset, double w, double[] e)
        {
            var p = new double[3];
            var a = new double[3];
            ForwardKinematics(q, toolOffset, p, a);
            e[0] = tp[0] - p[0];
            e[1] = tp[1] - p[1];
            e[2] = tp[2] - p[2];
            if (ta != null && e.Length >= 6)
            {
                // Rotation needed to turn the current approach axis onto the target one: a x ta.
                e[3] = w * (a[1] * ta[2] - a[2] * ta[1]);
                e[4] = w * (a[2] * ta[0] - a[0] * ta[2]);
                e[5] = w * (a[0] * ta[1] - a[1] * ta[0]);
            }
        }

        // ---------- small matrix helpers (4x4 homogeneous, row-major) ----------

        static double[,] Identity()
        {
            var m = new double[4, 4];
            for (int i = 0; i < 4; i++) m[i, i] = 1;
            return m;
        }

        static double[,] Pose(double x, double y, double z, double roll, double pitch, double yaw)
        {
            // URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
            double cr = Math.Cos(roll), sr = Math.Sin(roll);
            double cp = Math.Cos(pitch), sp = Math.Sin(pitch);
            double cy = Math.Cos(yaw), sy = Math.Sin(yaw);
            var m = Identity();
            m[0, 0] = cy * cp; m[0, 1] = cy * sp * sr - sy * cr; m[0, 2] = cy * sp * cr + sy * sr;
            m[1, 0] = sy * cp; m[1, 1] = sy * sp * sr + cy * cr; m[1, 2] = sy * sp * cr - cy * sr;
            m[2, 0] = -sp;     m[2, 1] = cp * sr;                m[2, 2] = cp * cr;
            m[0, 3] = x; m[1, 3] = y; m[2, 3] = z;
            return m;
        }

        static double[,] RotZ(double a)
        {
            var m = Identity();
            double c = Math.Cos(a), s = Math.Sin(a);
            m[0, 0] = c; m[0, 1] = -s;
            m[1, 0] = s; m[1, 1] = c;
            return m;
        }

        static double[,] Mul(double[,] a, double[,] b)
        {
            var m = new double[4, 4];
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                {
                    double s = 0;
                    for (int k = 0; k < 4; k++) s += a[r, k] * b[k, c];
                    m[r, c] = s;
                }
            return m;
        }

        // Gaussian elimination with partial pivoting. Returns null if the matrix is singular.
        static double[] SolveLinear(double[,] a, double[] b)
        {
            int n = b.Length;
            var m = (double[,])a.Clone();
            var x = (double[])b.Clone();
            for (int col = 0; col < n; col++)
            {
                int pivot = col;
                for (int r = col + 1; r < n; r++)
                    if (Math.Abs(m[r, col]) > Math.Abs(m[pivot, col])) pivot = r;
                if (Math.Abs(m[pivot, col]) < 1e-12) return null;
                if (pivot != col)
                {
                    for (int c = 0; c < n; c++) { double t = m[col, c]; m[col, c] = m[pivot, c]; m[pivot, c] = t; }
                    double tb = x[col]; x[col] = x[pivot]; x[pivot] = tb;
                }
                for (int r = col + 1; r < n; r++)
                {
                    double f = m[r, col] / m[col, col];
                    for (int c = col; c < n; c++) m[r, c] -= f * m[col, c];
                    x[r] -= f * x[col];
                }
            }
            for (int r = n - 1; r >= 0; r--)
            {
                double s = x[r];
                for (int c = r + 1; c < n; c++) s -= m[r, c] * x[c];
                x[r] = s / m[r, r];
            }
            return x;
        }

        static double Norm(double[] v)
        {
            double s = 0;
            foreach (double d in v) s += d * d;
            return Math.Sqrt(s);
        }

        static double Clamp(double v, double lo, double hi) => v < lo ? lo : (v > hi ? hi : v);
    }
}
