using bluebean.Physics.PBD.DataStruct;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct ApplyBatchedCollisionConstraintsBatchJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<BurstContact> contacts;

        [NativeDisableParallelForRestriction] public NativeArray<float4> positions;
        [NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
        [NativeDisableParallelForRestriction] public NativeArray<int> counts;

        [ReadOnly] public BatchData batchData;

        public void Execute(int workItemIndex)
        {
            int start, end;
            batchData.GetConstraintRange(workItemIndex, out start, out end);

            for (int i = start; i < end; ++i)
            {
                //int simplexStartA = simplexCounts.GetSimplexStartAndSize(contacts[i].bodyA, out int simplexSizeA);
                //int simplexStartB = simplexCounts.GetSimplexStartAndSize(contacts[i].bodyB, out int simplexSizeB);

                //for (int j = 0; j < simplexSizeA; ++j)
                //{
                //    int particleIndex = simplices[simplexStartA + j];
                //    BurstConstraintsBatchImpl.ApplyPositionDelta(particleIndex, constraintParameters.SORFactor, ref positions, ref deltas, ref counts);
                //    BurstConstraintsBatchImpl.ApplyOrientationDelta(particleIndex, constraintParameters.SORFactor, ref orientations, ref orientationDeltas, ref orientationCounts);
                //}

                //for (int j = 0; j < simplexSizeB; ++j)
                //{
                //    int particleIndex = simplices[simplexStartB + j];
                //    BurstConstraintsBatchImpl.ApplyPositionDelta(particleIndex, constraintParameters.SORFactor, ref positions, ref deltas, ref counts);
                //    BurstConstraintsBatchImpl.ApplyOrientationDelta(particleIndex, constraintParameters.SORFactor, ref orientations, ref orientationDeltas, ref orientationCounts);
                //}

            }

        }

    }

}
