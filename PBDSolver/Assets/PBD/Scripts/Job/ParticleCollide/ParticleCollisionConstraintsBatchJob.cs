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
        [ReadOnly] public NativeArray<float> invMasses;
        [ReadOnly] public NativeArray<float4> radii;

        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> positions;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<BurstContact> contacts;

        [ReadOnly] public float4 gravity;
        [ReadOnly] public float substepTime;

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

                float4 simplexPositionA = float4.zero, simplexPositionB = float4.zero;
                float simplexRadiusA = 0, simplexRadiusB = 0;

                //todo
                simplexPositionA = positions[particleA];



                float4 posA = simplexPositionA - contact.normal * simplexRadiusA;
                float4 posB = simplexPositionB + contact.normal * simplexRadiusB;

                // adhesion:
                float lambda = contact.SolveAdhesion(posA, posB, 0, 0, substepTime);

                // depenetration:
                lambda += contact.SolvePenetration(posA, posB, 5.5f * substepTime);

                // Apply normal impulse to both particles (w/ shock propagation):
                if (math.abs(lambda) > BurstMath.epsilon)
                {
                    float shock = 1.0f * math.dot(contact.normal, math.normalizesafe(gravity));
                    float4 delta = lambda * contact.normal;

                    //for (int j = 0; j < simplexSizeA; ++j)
                    //{
                    //    int particleIndex = simplices[simplexStartA + j];
                    //    deltas[particleIndex] += delta * invMasses[particleIndex] * contact.pointA[j] * baryScale * (1 - shock);
                    //    counts[particleIndex]++;
                    //}

                    //baryScale = BurstMath.BaryScale(contact.pointB);
                    //for (int j = 0; j < simplexSizeB; ++j)
                    //{
                    //    int particleIndex = simplices[simplexStartB + j];
                    //    deltas[particleIndex] -= delta * invMasses[particleIndex] * contact.pointB[j] * baryScale * (1 + shock);
                    //    counts[particleIndex]++;
                    //}
                }

                // Apply position deltas immediately, if using sequential evaluation:
                //if (constraintParameters.evaluationOrder == Oni.ConstraintParameters.EvaluationOrder.Sequential)
                //{
                //    for (int j = 0; j < simplexSizeA; ++j)
                //    {
                //        int particleIndex = simplices[simplexStartA + j];
                //        BurstConstraintsBatchImpl.ApplyPositionDelta(particleIndex, constraintParameters.SORFactor, ref positions, ref deltas, ref counts);
                //    }

                //    for (int j = 0; j < simplexSizeB; ++j)
                //    {
                //        int particleIndex = simplices[simplexStartB + j];
                //        BurstConstraintsBatchImpl.ApplyPositionDelta(particleIndex, constraintParameters.SORFactor, ref positions, ref deltas, ref counts);
                //    }
                //}

                contacts[i] = contact;
            }
        }


    }
}