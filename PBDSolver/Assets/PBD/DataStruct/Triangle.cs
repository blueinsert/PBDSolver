using bluebean.UGFramework.DataStruct;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace bluebean.UGFramework.DataStruct
{
    public struct Triangle : IBounded
    {
        public int i1;
        public int i2;
        public int i3;

        Aabb b;

        public Triangle(int i1, int i2, int i3, Vector3 v1, Vector3 v2, Vector3 v3)
        {
            this.i1 = i1;
            this.i2 = i2;
            this.i3 = i3;
            b = new Aabb(v1);
            b.Encapsulate(v2);
            b.Encapsulate(v3);
        }

        public Aabb GetBounds()
        {
            return b;
        }
    }
}
