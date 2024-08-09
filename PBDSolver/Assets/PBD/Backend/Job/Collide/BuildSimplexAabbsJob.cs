using bluebean.Physics.PBD.DataStruct;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct
{
    [BurstCompile]
    struct BuildSimplexAabbsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float4> radii;
        //[ReadOnly] public NativeArray<float> fluidRadii;
        [ReadOnly] public NativeArray<float4> positions;
        [ReadOnly] public NativeArray<float4> velocities;

        // simplex arrays:
        [ReadOnly] public NativeArray<int> simplices;
        //[ReadOnly] public SimplexCounts simplexCounts;

        //[ReadOnly] public NativeArray<int> particleMaterialIndices;
        //[ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;
        [ReadOnly] public float collisionMargin;
        [ReadOnly] public float continuousCollisionDetection;
        [ReadOnly] public float dt;

        public NativeArray<BurstAabb> simplexBounds;

        public void Execute(int i)
        {
            int simplexStart = i;// simplexCounts.GetSimplexStartAndSize(i, out int simplexSize);

            var bounds = new BurstAabb(float.MaxValue, float.MinValue);

            {
                int p = simplices[simplexStart];

                // Find this particle's stick distance:
                //int m = particleMaterialIndices[p];
                float stickDistance = 0;// m >= 0 ? collisionMaterials[m].stickDistance : 0;

                // Expand simplex bounds, using both the particle's original position and its velocity:
                bounds.EncapsulateParticle(positions[p], positions[p] + velocities[p] * continuousCollisionDetection * dt,
                                            math.max(radii[p].x + stickDistance, 0.0f) + collisionMargin);
            }

            simplexBounds[i] = bounds;
        }
    }
}
