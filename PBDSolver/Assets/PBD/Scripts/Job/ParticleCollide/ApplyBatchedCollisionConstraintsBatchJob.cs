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
                var contact = contacts[i];
                int particleA = contact.bodyA;
                int particleB = contact.bodyB;

                if (counts[particleA] > 0)
                {
                    positions[particleA] += deltas[particleA] * 1.0f / counts[particleA];
                    deltas[particleA] = float4.zero;
                    counts[particleA] = 0;
                }

                if (counts[particleB] > 0)
                {
                    positions[particleB] += deltas[particleB] * 1.0f / counts[particleB];
                    deltas[particleB] = float4.zero;
                    counts[particleB] = 0;
                }

            }

        }

    }

}
