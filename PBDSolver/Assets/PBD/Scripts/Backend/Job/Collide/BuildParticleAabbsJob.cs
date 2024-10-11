using bluebean.Physics.PBD.DataStruct;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    /// <summary>
    /// ���������ٶȣ�deltaTime�ȸ�����һ֡�����ӵĿ��ܻ��Χ
    /// </summary>
    [BurstCompile]
    public struct BuildParticleAabbsJob : IJobParallelFor
    {
        //����
        [ReadOnly] public NativeArray<float4> radii;
        [ReadOnly] public NativeArray<float4> positions;
        [ReadOnly] public NativeArray<float4> velocities;
        [ReadOnly] public float collisionMargin;
        [ReadOnly] public float continuousCollisionDetection;
        [ReadOnly] public float dt;

        //���
        public NativeArray<BurstAabb> simplexBounds;

        public void Execute(int i)
        {
            var bounds = new BurstAabb(float.MaxValue, float.MinValue);

            {
                int p = i;

                // Find this particle's stick distance:
                //int m = particleMaterialIndices[p];
                float stickDistance = 0;// m >= 0 ? collisionMaterials[m].stickDistance : 0;

                // Expand simplex bounds, using both the particle's original position and its velocity:
                bounds.EncapsulateParticle(positions[p], positions[p] + velocities[p] * continuousCollisionDetection * dt,
                                            math.max(radii[p].x + stickDistance, 0.0f) + collisionMargin);
            }

            simplexBounds[i] = bounds;
        }
    }
}
