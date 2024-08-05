using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

namespace bluebean.UGFramework.DataStruct
{
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct VectorInt2
    {
        public int x;
        public int y;

        public VectorInt2(int x, int y)
        {
            this.x = x;
            this.y = y;
        }

        public VectorInt2(int x)
        {
            this.x = x;
            this.y = x;
        }
    }
}
