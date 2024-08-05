using bluebean.UGFramework.Physics;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

[BurstCompile]
struct PredictPositionsJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float4> m_externalForces;
    [ReadOnly] public NativeArray<float> m_inverseMasses;
    [ReadOnly] public NativeArray<float4> m_particleProperties;
    [ReadOnly] public float4 m_gravity;
    [ReadOnly] public float m_deltaTime;

    [NativeDisableParallelForRestriction] public NativeArray<float4> m_positions;
    [NativeDisableParallelForRestriction] public NativeArray<float4> m_prevPositions;
    [NativeDisableParallelForRestriction] public NativeArray<float4> m_velocities;

    public void Execute(int index)
    {
        int i = index;
        m_prevPositions[i] = m_positions[i];
        float4 property = m_particleProperties[i];
        float4 vel = m_velocities[i] + (m_inverseMasses[i] * m_externalForces[i] + m_gravity) * m_deltaTime;
        if (!PBDUtil.IsParticleFixed(property))
        {
            m_velocities[i] = vel;
            m_positions[i] = m_positions[i] + m_velocities[i] * m_deltaTime;
        }  
    }
}
