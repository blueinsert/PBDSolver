using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct ApplyShapeMatchingConstraintsBatchJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> particleIndices;
        [ReadOnly] public NativeArray<int> firstIndex;
        [ReadOnly] public NativeArray<int> numIndices;
        [ReadOnly] public float sorFactor;

        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> positions;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

        public void Execute(int i)
        {
            int first = firstIndex[i];
            int last = first + numIndices[i];

            for (int k = first; k < last; ++k)
            {
                int p = particleIndices[k];
                if (counts[p] > 0)
                {
                    positions[p] += deltas[p] * sorFactor / counts[p];
                    deltas[p] = float4.zero;
                    counts[p] = 0;
                }
            }
        }
    }
}