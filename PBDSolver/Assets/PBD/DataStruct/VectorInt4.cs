using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

namespace bluebean.UGFramework.DataStruct
{
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct VectorInt4
    {
        public int x;
        public int y;
        public int z;
        public int w;

        public VectorInt4(int x, int y, int z, int w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        public VectorInt4(int x)
        {
            this.x = x;
            this.y = x;
            this.z = x;
            this.w = x;
        }

        public int this[int index]
        {
            get
            {
                // 检查索引是否越界  
                if (index < 0 || index >= 4)
                {
                    throw new IndexOutOfRangeException("Index was out of range.");
                }
                switch (index)
                {
                    case 0: return x;
                    case 1: return y;
                    case 2: return z;
                    case 3: return w;
                }
                return -1;
            }
            set
            {
                // 检查索引是否越界  
                if (index < 0 || index >= 4)
                {
                    throw new IndexOutOfRangeException("Index was out of range.");
                }
                switch (index)
                {
                    case 0: x = value; break;
                    case 1: y = value; break;
                    case 2: z = value; break;
                    case 3: w = value; break;
                }
            }
        }
    }
}
