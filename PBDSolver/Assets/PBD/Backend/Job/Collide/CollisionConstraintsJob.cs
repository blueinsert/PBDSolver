using bluebean.Physics.PBD.DataStruct;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct CollisionConstraintsJob : IJob
    {
        [ReadOnly] public NativeArray<float4> prevPositions;
        //[ReadOnly] public NativeArray<quaternion> orientations;
        //[ReadOnly] public NativeArray<quaternion> prevOrientations;
        [ReadOnly] public NativeArray<float> invMasses;
        [ReadOnly] public NativeArray<float4> radii;
        //[ReadOnly] public NativeArray<int> particleMaterialIndices;

        // simplex arrays:
        //[ReadOnly] public NativeArray<int> simplices;
        //[ReadOnly] public SimplexCounts simplexCounts;

        [ReadOnly] public NativeArray<BurstColliderShape> shapes;
        [ReadOnly] public NativeArray<BurstAffineTransform> transforms;
        //[ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;
        //[ReadOnly] public NativeArray<BurstRigidbody> rigidbodies;
        //public NativeArray<float4> rigidbodyLinearDeltas;
        //public NativeArray<float4> rigidbodyAngularDeltas;

        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> positions;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

        public NativeArray<BurstContact> contacts;
        //[ReadOnly] public BurstInertialFrame inertialFrame;
        //[ReadOnly] public Oni.ConstraintParameters constraintParameters;
        //[ReadOnly] public Oni.SolverParameters solverParameters;
        [ReadOnly] public float stepTime;
        [ReadOnly] public float substepTime;
        [ReadOnly] public int substeps;

        public void Execute()
        {
            for (int i = 0; i < contacts.Length; ++i)
            {
                var contact = contacts[i];

                int simplexIndex = contact.bodyA;// simplexCounts.GetSimplexStartAndSize(contact.bodyA, out int simplexSize);
                int colliderIndex = contact.bodyB;

                // Skip contacts involving triggers:
                //if (shapes[colliderIndex].flags > 0)
                //    continue;

                // Get the rigidbody index (might be < 0, in that case there's no rigidbody present)
                //int rigidbodyIndex = shapes[colliderIndex].rigidbodyIndex;

                // Combine collision materials (use material from first particle in simplex)
                //BurstCollisionMaterial material = CombineCollisionMaterials(simplices[simplexIndex], colliderIndex);

                // Get relative velocity at contact point.
                // As we do not consider true ellipses for collision detection, particle contact points are never off-axis.
                // So particle angular velocity does not contribute to normal impulses, and we can skip it.
                float4 simplexPosition = positions[simplexIndex];
                float4 simplexPrevPosition = prevPositions[simplexIndex];
                float simplexRadius = radii[simplexIndex].x;
                float invMass = invMasses[simplexIndex];

                //for (int j = 0; j < simplexSize; ++j)
                //{
                //    int particleIndex = simplices[simplexIndex + j];
                //    simplexPosition += positions[particleIndex] * contact.pointA[j];
                //    simplexPrevPosition += prevPositions[particleIndex] * contact.pointA[j];
                //    simplexRadius += BurstMath.EllipsoidRadius(contact.normal, orientations[particleIndex], radii[particleIndex].xyz) * contact.pointA[j];
                //}

                // project position to the end of the full step:
                float4 posA = math.lerp(simplexPrevPosition, simplexPosition, substeps);
                posA += -contact.normal * simplexRadius;

                float4 posB = contact.pointB;

                //contact.CalculateContactMassesA(invMasses,);
                contact.normalInvMassA = contact.tangentInvMassA = contact.bitangentInvMassA = invMass;
                //if (rigidbodyIndex >= 0)
                //    posB += BurstMath.GetRigidbodyVelocityAtPoint(rigidbodyIndex, contact.pointB, rigidbodies, rigidbodyLinearDeltas, rigidbodyAngularDeltas, inertialFrame.frame) * stepTime;

                // adhesion:
                //float lambda = contact.SolveAdhesion(posA, posB, material.stickDistance, material.stickiness, stepTime);

                // depenetration:
                float lambda = contact.SolvePenetration(posA, posB, 10.1f * stepTime);

                // Apply normal impulse to both simplex and rigidbody:
                if (math.abs(lambda) > BurstMath.epsilon)
                {
                    float4 delta = lambda * contact.normal / substeps;

                    int particleIndex = simplexIndex;
                    deltas[particleIndex] += delta * invMasses[particleIndex];
                    counts[particleIndex]++;

                    //if (rigidbodyIndex >= 0)
                    //{
                    //    BurstMath.ApplyImpulse(rigidbodyIndex, -lambda / stepTime * contact.normal, contact.pointB, rigidbodies, rigidbodyLinearDeltas, rigidbodyAngularDeltas, inertialFrame.frame);
                    //}
                }

                contacts[i] = contact;
            }
        }

    }
}