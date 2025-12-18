using bluebean.Physics.PBD.DataStruct;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public interface ISolver
    {
        ColliderWorld ColliderWorld { get; }
        float StretchConstrainCompliance { get; }
        float VolumeConstrainCompliance { get; }

        #region 粒子数据获取接口
        NativeArray<float> ParticleRadius { get; }
        NativeArray<float4> ParticlePositions { get; }
        NativeArray<float4> PrevParticlePositions { get; }
        NativeArray<float4> ParticleVels { get; }
        public NativeArray<BurstAabb> ParticleAabb { get; }
        //每个粒子的网格坐标
        public NativeArray<int4> CellCoords { get; }
        public NativeArray<int> Groups { get; }
        NativeArray<float4> ParticleProperties { get; }
        NativeArray<float4> ExternalForces { get; }
        NativeArray<float> InvMasses { get; }
        NativeArray<float4> PositionDeltas { get; }
        NativeArray<float4> Gradients { get; }
        #endregion

        NativeArray<int> PositionConstraintCounts { get; }
        NativeArray<BurstContact> ColliderContacts { get; }

        NativeArray<BurstContact> ParticleContacts { get; }

        Vector3 Gravity { get; }

        void AddActor(PBDActor actor);

        Vector3 GetParticlePosition(int particleIndex);

        int GetParticleCount();

        void PushStretchConstrain(StretchConstrainData stretchConstrainData);

        void PushVolumeConstrain(VolumeConstrainData volumeConstrainData);

        void PushShapeMatchingConstrain(ShapeMatchingConstrainData constrainData);

        void ScheduleBatchedJobsIfNeeded();
    }
}
