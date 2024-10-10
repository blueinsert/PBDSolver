using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct
{
    public interface IDistanceFunction
    {
        void Evaluate(float4 point, float4 radii, quaternion orientation, ref SurfacePoint projectedPoint);
    }
}