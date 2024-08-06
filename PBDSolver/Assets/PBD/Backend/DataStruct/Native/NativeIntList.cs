using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct.Native
{
    [Serializable]
    public class NativeIntList : NativeList<int>
    {
        public NativeIntList() { }
        public NativeIntList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = 0;
        }

    }
}
