using bluebean.Physics;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

[BurstCompile]
struct UpdateVelJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float4> m_prevPositions;
    [ReadOnly] public float m_deltaTime;
    [ReadOnly] public float m_velDamping;
    [ReadOnly] public float m_sleepThreshold;

    [NativeDisableParallelForRestriction] public NativeArray<float4> m_positions;
    [NativeDisableParallelForRestriction] public NativeArray<float4> m_velocities;

    public void Execute(int index)
    {
        int i = index;
        m_velocities[i] = (m_positions[i] - m_prevPositions[i])/m_deltaTime;
        m_velocities[i] *= m_velDamping;

        if (math.lengthsq(m_velocities[i])*0.5f < m_sleepThreshold)
        {
            m_positions[i] = m_prevPositions[i];
            m_velocities[i] = float4.zero;
        }
    }
}

