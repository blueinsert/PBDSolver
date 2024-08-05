using System;
using Unity.Collections;
using UnityEngine;

namespace bluebean.UGFramework.DataStruct.Native
{
    [Serializable]
    public class NativeVector2List : NativeList<Vector2>
    {
        public NativeVector2List() { }
        public NativeVector2List(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = Vector2.zero;
        }

    }
}

