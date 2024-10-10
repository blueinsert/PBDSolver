using bluebean.Physics.PBD.DataStruct;
using System.Collections;
using System.Collections.Generic;
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
        NativeArray<float4> ParticleRadius { get; }
        NativeArray<float4> ParticlePositions { get; }
        NativeArray<float4> PrevParticlePositions { get; }
        NativeArray<float4> ParticleVels { get; }
        public NativeArray<BurstAabb> ParticleAabb { get; }
        NativeArray<float4> ParticleProperties { get; }
        NativeArray<float4> ExternalForces { get; }
        NativeArray<float> InvMasses { get; }
        NativeArray<float4> PositionDeltas { get; }
        NativeArray<float4> Gradients { get; }
        #endregion

        NativeArray<int> PositionConstraintCounts { get; }
        NativeArray<BurstContact> ColliderContacts { get; }

        void AddActor(PDBActor actor);

        Vector3 GetParticlePosition(int particleIndex);

        void PushStretchConstrain(StretchConstrainData stretchConstrainData);

        void PushVolumeConstrain(VolumeConstrainData volumeConstrainData);
    }
}
