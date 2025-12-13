using System;
using Unity.Collections;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeMatrix4x4List : NativeList<Matrix4x4>
    {
        public NativeMatrix4x4List(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = Matrix4x4.identity;
        }
    }
}