using bluebean.UGFramework.DataStruct;
using System;
using Unity.Collections;
using UnityEngine;

namespace bluebean.UGFramework.DataStruct.Native
{
    [Serializable]
    public class NativeBIHNodeList : NativeList<BIHNode>
    {
        public NativeBIHNodeList() { }
        public NativeBIHNodeList(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = new BIHNode();
        }
    }
}

