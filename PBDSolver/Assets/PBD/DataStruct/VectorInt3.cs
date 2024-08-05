using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

namespace bluebean.UGFramework.DataStruct
{
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct VectorInt3
    {
        public int x;
        public int y;
        public int z;

        public VectorInt3(int x, int y, int z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public VectorInt3(int x)
        {
            this.x = x;
            this.y = x;
            this.z = x;
        }
    }
}
