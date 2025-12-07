using bluebean.Physics.PBD.DataStruct;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct ApplyCollisionConstraintsJob : IJob
    {
        [ReadOnly] public NativeArray<BurstContact> contacts;

        [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
        [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
        [NativeDisableParallelForRestriction] public NativeArray<int> counts;

        public void Execute()
        {
            float sorFactor = 1f;
            for (int i = 0; i < contacts.Length; ++i)
            {
                int particleIndex = contacts[i].bodyA;
                {
                    if (counts[particleIndex] > 0)
                    {
                        positions[particleIndex] += deltas[particleIndex] * sorFactor / counts[particleIndex];
                        deltas[particleIndex] = float4.zero;
                        counts[particleIndex] = 0;
                    }
                }
            }
        }

    }
}
