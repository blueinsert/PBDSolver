using System;
using System.Runtime.InteropServices;

namespace bluebean.Physics.PBD.DataStruct
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
