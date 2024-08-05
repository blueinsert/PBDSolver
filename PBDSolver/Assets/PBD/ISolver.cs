using bluebean.UGFramework.DataStruct;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.UGFramework.Physics
{
    public interface ISolver
    {
        float StretchConstrainCompliance { get; }
        float VolumeConstrainCompliance { get; }
        NativeArray<float4> ParticlePositions { get; }
        NativeArray<float4> PrevParticlePositions { get; }
        NativeArray<float4> ParticleVels { get; }
        NativeArray<float4> ParticleProperties { get; }
        NativeArray<float4> ExternalForces { get; }
        NativeArray<float> InvMasses { get; }
        NativeArray<float4> PositionDeltas { get; }
        NativeArray<float4> Gradients { get; }
        NativeArray<int> PositionConstraintCounts { get; }

        void AddActor(PDBActor actor);

        Vector3 GetParticlePosition(int particleIndex);

        void PushStretchConstrain(StretchConstrainData stretchConstrainData);

        void PushVolumeConstrain(VolumeConstrainData volumeConstrainData);
    }
}
