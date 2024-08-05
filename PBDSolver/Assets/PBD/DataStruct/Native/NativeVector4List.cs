using bluebean.UGFramework.DataStruct;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.UGFramework.DataStruct.Native
{

    [Serializable]
    public class NativeVector4List : NativeList<Vector4>
    {
        public NativeVector4List() { }
        public NativeVector4List(int capacity = 8, int alignment = 16) : base(capacity, alignment)
        {
            for (int i = 0; i < capacity; ++i)
                this[i] = Vector4.zero;
        }


        public Vector3 GetVector3(int index)
        {
            unsafe
            {
                byte* start = (byte*)m_AlignedPtr + index * sizeof(Vector4);
                return *(Vector3*)start;
            }
        }

        public void SetVector3(int index, Vector3 value)
        {
            unsafe
            {
                byte* start = (byte*)m_AlignedPtr + index * sizeof(Vector4);
                *(Vector3*)start = value;
            }
        }
    }
}
