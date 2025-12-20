using System;
using System.Runtime.InteropServices;
using System.Text;

namespace bluebean.Physics.PBD.DataStruct
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
                // ��������Ƿ�Խ��  
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
                // ��������Ƿ�Խ��  
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

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            sb.Append("[");
            sb.Append(x);sb.Append(",");
            sb.Append(y);sb.Append(", ");
            sb.Append(z);sb.Append(",");
            sb.Append(w);
            sb.Append("]");
            return sb.ToString();
        }
    }
}
