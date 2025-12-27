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
    public struct FrictionConstraintsBatchJob : IJob
    {
        [ReadOnly] public NativeArray<float4> positions;
        [ReadOnly] public NativeArray<float4> prevPositions;

        [ReadOnly] public NativeArray<float> invMasses;
        [ReadOnly] public NativeArray<float> radii;

        [ReadOnly] public NativeArray<float> staticFrictions;
        [ReadOnly] public NativeArray<float> dynamicFrictions;

        [ReadOnly] public NativeArray<BurstColliderShape> shapes;
        [ReadOnly] public NativeArray<BurstAffineTransform> transforms;
        //[ReadOnly] public NativeArray<BurstRigidbody> rigidbodies;
        //public NativeArray<float4> rigidbodyLinearDeltas;
        //public NativeArray<float4> rigidbodyAngularDeltas;

        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

        public NativeArray<BurstContact> contacts;
        [ReadOnly] public float stepTime;
        [ReadOnly] public float substepTime;
        [ReadOnly] public int substeps;

        public void Execute()
        {
            for (int i = 0; i < contacts.Length; ++i)
            {
                var contact = contacts[i];

                // Get the indices of the particle and collider involved in this contact:
                int particleIndex = contact.bodyA;
                int colliderIndex = contact.bodyB;

                //int rigidbodyIndex = shapes[colliderIndex].rigidbodyIndex;

                // Calculate relative velocity:
                float4 rA = float4.zero, rB = float4.zero;

                float4 prevPositionA = prevPositions[particleIndex];
                float4 linearVelocityA = BurstIntegration.DifferentiateLinear(positions[particleIndex], prevPositions[particleIndex], substepTime);
                float simplexRadiusA = radii[particleIndex];

                float4 relativeVelocity = linearVelocityA;

                float staticFriction = staticFrictions[particleIndex];
                float dynamicFriction = dynamicFrictions[particleIndex];

                // Subtract rigidbody velocity:
                //if (rigidbodyIndex >= 0)
                {
                    // Note: unlike rA, that is expressed in solver space, rB is expressed in world space.
                    //rB = inertialFrame.frame.TransformPoint(contact.pointB) - rigidbodies[rigidbodyIndex].com;
                    //relativeVelocity -= BurstMath.GetRigidbodyVelocityAtPoint(rigidbodyIndex, contact.pointB, rigidbodies, rigidbodyLinearDeltas, rigidbodyAngularDeltas, inertialFrame.frame);
                }
                
                // Determine impulse magnitude:
                float2 impulses = contact.SolveFriction(relativeVelocity, staticFriction, dynamicFriction, stepTime);
                //Debug.Log($"contact.SolveFriction {relativeVelocity} {impulses}");

                if (math.abs(impulses.x) > BurstMath.epsilon || math.abs(impulses.y) > BurstMath.epsilon)
                {
                    float4 tangentImpulse = impulses.x * contact.tangent;
                    float4 bitangentImpulse = impulses.y * contact.bitangent;

                    deltas[particleIndex] += (tangentImpulse * contact.tangentInvMassA + bitangentImpulse * contact.bitangentInvMassA) * substepTime;
                    counts[particleIndex]++;

                    //if (rigidbodyIndex >= 0)
                    //{
                    //    BurstMath.ApplyImpulse(rigidbodyIndex, -totalImpulse, contact.pointB, rigidbodies, rigidbodyLinearDeltas, rigidbodyAngularDeltas, inertialFrame.frame);
                    //}
                }

                contacts[i] = contact;
            }
        }

    }
}
