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

                float4 simplexVelocityA = float4.zero;
                float4 simplexPrevPositionA = float4.zero;
                quaternion simplexPrevOrientationA = new quaternion(0, 0, 0, 0);
                float simplexRadiusA = 0;
                float simplexInvMassA = 0;
                float4 simplexInvInertiaA = float4.zero;

                float4 simplexVelocityB = float4.zero;
                float4 simplexPrevPositionB = float4.zero;
                quaternion simplexPrevOrientationB = new quaternion(0, 0, 0, 0);
                float simplexRadiusB = 0;
                float simplexInvMassB = 0;
                float4 simplexInvInertiaB = float4.zero;

                //todo
                simplexVelocityA = velocities[particleA];
                simplexPrevPositionA = prevPositions[particleA];

                // update contact distance
                float dAB = math.dot(simplexPrevPositionA - simplexPrevPositionB, contact.normal);
                contact.distance = dAB - (simplexRadiusA + simplexRadiusB);

                // calculate contact points:
                float4 contactPointA = simplexPrevPositionB + contact.normal * (contact.distance + simplexRadiusB);
                float4 contactPointB = simplexPrevPositionA - contact.normal * (contact.distance + simplexRadiusA);

                // update contact basis:
                contact.CalculateBasis(simplexVelocityA - simplexVelocityB);

                // update contact masses:
                bool rollingContacts = false; 
                contact.CalculateContactMassesA(simplexInvMassA, simplexInvInertiaA, simplexPrevPositionA, simplexPrevOrientationA, contactPointA, rollingContacts);
                contact.CalculateContactMassesB(simplexInvMassB, simplexInvInertiaB, simplexPrevPositionB, simplexPrevOrientationB, contactPointB, rollingContacts);

                contacts[i] = contact;
            }
        }
    }

}