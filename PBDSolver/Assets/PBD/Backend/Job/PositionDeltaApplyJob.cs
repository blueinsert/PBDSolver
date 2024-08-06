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
    public struct PositionDeltaApplyJob : IJobParallelFor
    {

        [ReadOnly] public NativeArray<float4> m_particleProperties;
        [ReadOnly] public float sorFactor;

        /// <summary>
        /// ����λ������
        /// </summary>
        [NativeDisableContainerSafetyRestriction]
        [NativeDisableParallelForRestriction]
        public NativeArray<float4> m_positions;
        /// <summary>
        /// ����Լ����������λ�ñ仯
        /// </summary>
        [NativeDisableContainerSafetyRestriction]
        [NativeDisableParallelForRestriction]
        public NativeArray<float4> m_deltas;
        /// <summary>
        /// ÿ�����㱻�ۼƴ���
        /// </summary>
        [NativeDisableContainerSafetyRestriction]
        [NativeDisableParallelForRestriction]
        public NativeArray<int> m_counts;


        public void Execute(int index)
        {         
            if (m_counts[index] > 0)
            {
                float4 property = m_particleProperties[index];
                if (!PBDUtil.IsParticleFixed(property))
                {
                    m_positions[index] += m_deltas[index] / m_counts[index];
                }
                m_deltas[index] = float4.zero;
                m_counts[index] = 0;
            }
        }
    }
}
