using System;
using System.Text;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeInt4List : NativeList<VectorInt4>
    {
        public NativeInt4List() { }
        public NativeInt4List(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new VectorInt4(0, 0, 0, 0);
        }

        public NativeInt4List(int capacity, int alignment, VectorInt4 defaultValue) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = defaultValue;
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            sb.Append("[");
            for (int i = 0; i < this.count; i++)
            {
                sb.Append(this[i].ToString());
            }
            sb.Append("]");
            return sb.ToString();
        }
    }
}

