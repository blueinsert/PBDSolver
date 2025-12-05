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

        //[ReadOnly] public NativeArray<int> simplices;
        //[ReadOnly] public SimplexCounts simplexCounts;

        [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
        [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
        [NativeDisableParallelForRestriction] public NativeArray<int> counts;

        //[NativeDisableParallelForRestriction] public NativeArray<quaternion> orientations;
        //[NativeDisableParallelForRestriction] public NativeArray<quaternion> orientationDeltas;
        //[NativeDisableParallelForRestriction] public NativeArray<int> orientationCounts;

        //[ReadOnly] public Oni.ConstraintParameters constraintParameters;

        public void Execute()
        {
            float sorFactor = 1f;
            for (int i = 0; i < contacts.Length; ++i)
            {
                int simplexIndex = contacts[i].bodyA;// simplexCounts.GetSimplexStartAndSize(contacts[i].bodyA, out int simplexSize);
                {
                    int particleIndex = simplexIndex;
                    if (counts[particleIndex] > 0)
                    {
                        positions[particleIndex] += deltas[particleIndex] * sorFactor / counts[particleIndex];
                        deltas[particleIndex] = float4.zero;
                        counts[particleIndex] = 0;
                    }
                    //BurstConstraintsBatchImpl.ApplyPositionDelta(particleIndex, constraintParameters.SORFactor, ref positions, ref deltas, ref counts);
                    //BurstConstraintsBatchImpl.ApplyOrientationDelta(particleIndex, constraintParameters.SORFactor, ref orientations, ref orientationDeltas, ref orientationCounts);
                }
            }
        }

    }
}
