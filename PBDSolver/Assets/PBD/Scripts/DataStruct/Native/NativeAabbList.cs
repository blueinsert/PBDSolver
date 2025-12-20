using System;
using System.Text;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeAabbList : NativeList<Aabb>
    {
        public NativeAabbList() { }
        public NativeAabbList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new Aabb();
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            sb.Append("[");
            for(int i = 0; i < this.count; i++)
            {
                sb.Append(this[i].ToString());
            }
            sb.Append("]");
            return sb.ToString();
        }

    }
}

