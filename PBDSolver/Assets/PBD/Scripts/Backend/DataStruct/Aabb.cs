using bluebean.Physics;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct
{
    public struct Aabb
    {
        public Vector4 min;
        public Vector4 max;

        public Vector4 center
        {
            get { return min + (max - min) * 0.5f; }
        }

        public Vector4 size
        {
            get { return max - min; }
        }

        public Aabb(Vector4 min, Vector4 max)
        {
            this.min = min;
            this.max = max;
        }

        public Aabb(Vector4 point)
        {
            this.min = point;
            this.max = point;
        }

        public void Encapsulate(Vector4 point)
        {
            min = Vector4.Min(min, point);
            max = Vector4.Max(max, point);
        }

        public void Encapsulate(Aabb other)
        {
            min = Vector4.Min(min, other.min);
            max = Vector4.Max(max, other.max);
        }

        public void FromBounds(Bounds bounds, float thickness, bool is2D = false)
        {
            Vector3 s = Vector3.one * thickness;
            min = bounds.min - s;
            max = bounds.max + s;
            if (is2D)
                max[2] = min[2] = 0;
        }

        public bool IntersectsAabb(in Aabb bounds, bool in2D = false)
        {
            if (in2D)
                return (min[0] <= bounds.max[0] && max[0] >= bounds.min[0]) &&
                       (min[1] <= bounds.max[1] && max[1] >= bounds.min[1]);

            return (min[0] <= bounds.max[0] && max[0] >= bounds.min[0]) &&
                   (min[1] <= bounds.max[1] && max[1] >= bounds.min[1]) &&
                   (min[2] <= bounds.max[2] && max[2] >= bounds.min[2]);
        }

        public void GetCorners(Vector3[] corners)
        {
            var size = this.size.XYZ();
            var p0 = min.XYZ();
            var p1 = p0 + new Vector3(0, 0, size.z);
            var p2 = p0 + new Vector3(size.x, 0, size.z);
            var p3 = p0 + new Vector3(size.x, 0, 0);
            var p4 = p0 + new Vector3(0, size.y, 0);
            var p5 = p1 + new Vector3(0, size.y, 0);
            var p6 = p2 + new Vector3(0, size.y, 0);
            var p7 = p3 + new Vector3(0, size.y, 0);
            corners[0] = p0;
            corners[1] = p1;
            corners[2] = p2;
            corners[3] = p3;
            corners[4] = p4;
            corners[5] = p5;
            corners[6] = p6;
            corners[7] = p7;
        }
    }
}
