using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace bluebean.UGFramework.Physics
{
    public static class PhysicsUtil
    {
        public const float epsilon = 0.0000001f;
        public const float sqrt3 = 1.73205080f;
        public const float sqrt2 = 1.41421356f;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int PureSign(this float val)
        {
            return ((0 <= val) ? 1 : 0) - ((val < 0) ? 1 : 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Swap<T>(ref T lhs, ref T rhs)
        {
            T temp = lhs;
            lhs = rhs;
            rhs = temp;
        }

        public static Bounds Transform(this Bounds b, Matrix4x4 m)
        {
            var xa = m.GetColumn(0) * b.min.x;
            var xb = m.GetColumn(0) * b.max.x;

            var ya = m.GetColumn(1) * b.min.y;
            var yb = m.GetColumn(1) * b.max.y;

            var za = m.GetColumn(2) * b.min.z;
            var zb = m.GetColumn(2) * b.max.z;

            Bounds result = new Bounds();
            Vector3 pos = m.GetColumn(3);
            result.SetMinMax(Vector3.Min(xa, xb) + Vector3.Min(ya, yb) + Vector3.Min(za, zb) + pos,
                             Vector3.Max(xa, xb) + Vector3.Max(ya, yb) + Vector3.Max(za, zb) + pos);


            return result;
        }
    }
}
