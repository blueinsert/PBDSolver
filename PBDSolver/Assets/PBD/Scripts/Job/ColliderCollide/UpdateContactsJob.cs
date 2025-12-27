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
    public struct UpdateContactsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float4> prevPositions;
        [ReadOnly] public NativeArray<float4> velocities;
        [ReadOnly] public NativeArray<float> radii;
        [ReadOnly] public NativeArray<float> invMasses;

        [ReadOnly] public NativeArray<BurstColliderShape> shapes;
        [ReadOnly] public NativeArray<BurstAffineTransform> transforms;
        //[ReadOnly] public NativeArray<BurstRigidbody> rigidbodies;
        //[ReadOnly] public NativeArray<float4> rigidbodyLinearDeltas;
        //[ReadOnly] public NativeArray<float4> rigidbodyAngularDeltas;

        public NativeArray<BurstContact> contacts;

        public void Execute(int i)
        {
            var contact = contacts[i];

            int particleIndex = contact.bodyA;

            float4 relativeVelocity = velocities[particleIndex];
            float4 particlePrevPosition = prevPositions[particleIndex];
            float particleInvMass = invMasses[particleIndex];
            float particleRadius = radii[particleIndex];


            // if there's a rigidbody present, subtract its velocity from the relative velocity:
            //int rigidbodyIndex = shapes[contact.bodyB].rigidbodyIndex;
            //if (rigidbodyIndex >= 0)
            //{
            //    relativeVelocity -= BurstMath.GetRigidbodyVelocityAtPoint(rigidbodyIndex, contact.pointB, rigidbodies, rigidbodyLinearDeltas, rigidbodyAngularDeltas, inertialFrame.frame);
            //}

            // update contact distance
            contact.distance = math.dot(particlePrevPosition - contact.pointB, contact.normal) - particleRadius;

            // calculate contact point in A's surface:
            float4 contactPoint = contact.pointB + contact.normal * contact.distance;

            // update contact orthonormal basis:
            contact.CalculateBasis(relativeVelocity);

            // calculate A's contact mass.
            contact.CalculateContactMassesA(particleInvMass, particlePrevPosition, contactPoint);

            // calculate B's contact mass.
            //if (rigidbodyIndex >= 0)
            //    contact.CalculateContactMassesB(rigidbodies[rigidbodyIndex], inertialFrame.frame);

            contacts[i] = contact;
        }
    }
}
