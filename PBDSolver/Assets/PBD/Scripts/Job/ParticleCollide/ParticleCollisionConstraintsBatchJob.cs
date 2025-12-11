using bluebean.Physics.PBD.DataStruct;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct ParticleCollisionConstraintsBatchJob : IJobParallelFor
    {
        // ‰»Î
        [ReadOnly] public NativeArray<float> invMasses;
        [ReadOnly] public NativeArray<float> radii;
        [ReadOnly] public float4 gravity;
        [ReadOnly] public float substepTime;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<BurstContact> contacts;
        [ReadOnly] public BatchData batchData;

        // ‰≥ˆ
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> positions;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;


        public void Execute(int workItemIndex)
        {
            int start, end;
            batchData.GetConstraintRange(workItemIndex, out start, out end);

            for (int i = start; i < end; ++i)
            {
                var contact = contacts[i];
                int particleA = contact.bodyA;
                int particleB = contact.bodyB;

                float4 positionA = float4.zero, positionB = float4.zero;
                float radiusA = 0, radiusB = 0;

                positionA = positions[particleA];
                radiusA = radii[particleA];
                positionB = positions[particleB];
                radiusB = radii[particleB];

                float4 posA = positionA - contact.normal * radiusA;
                float4 posB = positionB + contact.normal * radiusB;

                // adhesion:
                float lambda = contact.SolveAdhesion(posA, posB, 0, 0, substepTime);

                // depenetration:
                lambda += contact.SolvePenetration(posA, posB, 5.5f * substepTime);

                // Apply normal impulse to both particles (w/ shock propagation):
                if (math.abs(lambda) > BurstMath.epsilon)
                {
                    float shock = 1.0f * math.dot(contact.normal, math.normalizesafe(gravity));
                    float4 delta = lambda * contact.normal;

                    deltas[particleA] += delta * invMasses[particleA]* (1 - shock);
                    counts[particleA]++;

                    deltas[particleB] -= delta * invMasses[particleB] * (1 - shock);
                    counts[particleB]++;

                }

                contacts[i] = contact;
            }
        }


    }
}