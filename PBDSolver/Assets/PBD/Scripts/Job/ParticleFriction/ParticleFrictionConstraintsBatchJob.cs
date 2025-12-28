using bluebean.Physics.PBD.DataStruct;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct ParticleFrictionConstraintsBatchJob : IJobParallelFor
    {

        [ReadOnly] public NativeArray<float4> positions;
        [ReadOnly] public NativeArray<float4> prevPositions;

        [ReadOnly] public NativeArray<float> invMasses;
        [ReadOnly] public NativeArray<float> radii;
        [ReadOnly] public NativeArray<float> staticFrictions;
        [ReadOnly] public NativeArray<float> dynamicFrictions;

        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<BurstContact> contacts;

        [ReadOnly] public BatchData batchData;
        [ReadOnly] public float substepTime;

        public void Execute(int workItemIndex)
        {
            int start, end;
            batchData.GetConstraintRange(workItemIndex, out start, out end);

            for (int i = start; i < end; ++i)
            {
                var contact = contacts[i];

                int particleA = contact.bodyA;
                int particleB = contact.bodyB;

                float4 linearVelocityA = BurstIntegration.DifferentiateLinear(positions[particleA], prevPositions[particleA], substepTime);

                float4 linearVelocityB = BurstIntegration.DifferentiateLinear(positions[particleB], prevPositions[particleB], substepTime);

                // Calculate relative velocity:
                float4 relativeVelocity = linearVelocityA - linearVelocityB;

                float staticFriction = staticFrictions[particleA];
                float dynamicFriction = dynamicFrictions[particleA];

                // Calculate friction impulses (in the tangent and bitangent ddirections):
                float2 impulses = contact.SolveFriction(relativeVelocity, staticFriction, dynamicFriction, substepTime);

                // Apply friction impulses to both particles:
                if (math.abs(impulses.x) > BurstMath.epsilon || math.abs(impulses.y) > BurstMath.epsilon)
                {
                    float4 tangentImpulse = impulses.x * contact.tangent;
                    float4 bitangentImpulse = impulses.y * contact.bitangent;
                    float4 totalImpulse = tangentImpulse + bitangentImpulse;

                    deltas[particleA] += (tangentImpulse * contact.tangentInvMassA + bitangentImpulse * contact.bitangentInvMassA) * substepTime;
                    counts[particleA]++;

                    deltas[particleB] -= (tangentImpulse * contact.tangentInvMassB + bitangentImpulse * contact.bitangentInvMassB) * substepTime;
                    counts[particleB]++;
                }

                contacts[i] = contact;
            }
        }
    }
}