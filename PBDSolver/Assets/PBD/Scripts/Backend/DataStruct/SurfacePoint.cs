using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct
{
    public struct SurfacePoint
    {
        public float4 bary;
        public float4 point;
        public float4 normal;
    }
}
