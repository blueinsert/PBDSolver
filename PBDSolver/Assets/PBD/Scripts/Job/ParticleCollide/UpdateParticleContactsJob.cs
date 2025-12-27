using bluebean.Physics.PBD.DataStruct;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct UpdateParticleContactsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float4> prevPositions;
        [ReadOnly] public NativeArray<float4> velocities;
        [ReadOnly] public NativeArray<float> radii;
        [ReadOnly] public NativeArray<float> invMasses;

        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<BurstContact> contacts;

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

                float4 velocityA = float4.zero;
                float4 prevPositionA = float4.zero;
                float radiusA = 0;
                float invMassA = 0;

                float4 velocityB = float4.zero;
                float4 prevPositionB = float4.zero;
                float radiusB = 0;
                float invMassB = 0;

                velocityA = velocities[particleA];
                prevPositionA = prevPositions[particleA];
                radiusA = radii[particleA];
                invMassA = invMasses[particleA];

                velocityB = velocities[particleB];
                prevPositionB = prevPositions[particleB];
                radiusB = radii[particleB];
                invMassB = invMasses[particleB];

                // update contact distance
                float dAB = math.dot(prevPositionA - prevPositionB, contact.normal);
                contact.distance = dAB - (radiusA + radiusB);

                // calculate contact points:
                float4 contactPointA = prevPositionB + contact.normal * (contact.distance + radiusB);
                float4 contactPointB = prevPositionA - contact.normal * (contact.distance + radiusA);

                // update contact basis:
                contact.CalculateBasis(velocityA - velocityB);

                // update contact masses:
                contact.CalculateContactMassesA(invMassA, prevPositionA, contactPointA);
                contact.CalculateContactMassesB(invMassB, prevPositionB, contactPointB);

                contacts[i] = contact;
            }
        }
    }

}