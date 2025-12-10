using bluebean.Physics.PBD.DataStruct;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct CollisionConstraintsJob : IJob
    {
        [ReadOnly] public NativeArray<float4> prevPositions;
        [ReadOnly] public NativeArray<float> invMasses;
        [ReadOnly] public NativeArray<float> radii;

        [ReadOnly] public NativeArray<BurstColliderShape> shapes;
        [ReadOnly] public NativeArray<BurstAffineTransform> transforms;
        //[ReadOnly] public NativeArray<BurstRigidbody> rigidbodies;
        //public NativeArray<float4> rigidbodyLinearDeltas;
        //public NativeArray<float4> rigidbodyAngularDeltas;

        [ReadOnly] public float stepTime;
        [ReadOnly] public float substepTime;
        /// <summary>
        /// 当前step剩余的subStep迭代数量
        /// </summary>
        [ReadOnly] public int substeps;

        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> positions;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

        public NativeArray<BurstContact> contacts;

        public void Execute()
        {
            for (int i = 0; i < contacts.Length; ++i)
            {
                var contact = contacts[i];

                int particleIndex = contact.bodyA;// simplexCounts.GetSimplexStartAndSize(contact.bodyA, out int simplexSize);
                int colliderIndex = contact.bodyB;

                //这个substep的预测位置
                float4 particlePosition = positions[particleIndex];
                //上个substep的位置
                float4 particlePrevPosition = prevPositions[particleIndex];
                float particleRadius = radii[particleIndex];
                float invMass = invMasses[particleIndex];

                //外插值，用线性速度外插得到这个step结束时的位置
                // project position to the end of the full step:
                float4 posA = math.lerp(particlePrevPosition, particlePosition, substeps);
                //粒子表面距离碰撞点的最近点
                posA += -contact.normal * particleRadius;

                //碰撞点，碰撞体表面上的点，世界坐标系
                float4 posB = contact.pointB;

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
                    //除以substeps得到当前substep需要更新的delta
                    float4 delta = lambda * contact.normal / substeps;

                    //冲量直接改变位置
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