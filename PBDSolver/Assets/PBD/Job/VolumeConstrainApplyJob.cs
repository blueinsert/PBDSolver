using bluebean.UGFramework.DataStruct;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.UGFramework.Physics
{

    /// <summary>
    /// 汇总
    /// </summary>
    [BurstCompile]
    public struct VolumeConstrainSummarizeJob : IJobParallelFor
    {

        [ReadOnly] public NativeArray<int4> m_tets;
        [ReadOnly] public NativeArray<float4> m_positionDeltasPerConstrain;

        /// <summary>
        /// 本次约束求解产生的位置变化
        /// </summary>
        [NativeDisableContainerSafetyRestriction]
        [NativeDisableParallelForRestriction]
        public NativeArray<float4> m_deltas;
        /// <summary>
        /// 每个顶点被累计次数
        /// </summary>
        [NativeDisableContainerSafetyRestriction]
        [NativeDisableParallelForRestriction]
        public NativeArray<int> m_counts;

        public void Execute(int index)
        {
            int constrainIndex = index;
            var tet = m_tets[constrainIndex];
            var startIndex = constrainIndex * 4;
            for (int j = 0; j < 4; j++)
            {
                int p = tet[j];
                m_deltas[p] += m_positionDeltasPerConstrain[startIndex+j];
                m_counts[p]++;
            }
        }
    }

    /// <summary>
    /// 汇总
    /// </summary>
    [BurstCompile]
    public struct VolumeConstrainSummarizeJobSequential : IJob
    {

        [ReadOnly] public NativeArray<int4> m_tets;
        [ReadOnly] public NativeArray<float4> m_positionDeltasPerConstrain;

        /// <summary>
        /// 本次约束求解产生的位置变化
        /// </summary>
        [NativeDisableContainerSafetyRestriction]
        [NativeDisableParallelForRestriction]
        public NativeArray<float4> m_deltas;
        /// <summary>
        /// 每个顶点被累计次数
        /// </summary>
        [NativeDisableContainerSafetyRestriction]
        [NativeDisableParallelForRestriction]
        public NativeArray<int> m_counts;

        public void Execute()
        {
            for (int i = 0; i < m_tets.Length; i++)
            {
                int constrainIndex = i;
                var tet = m_tets[constrainIndex];
                var startIndex = constrainIndex * 4;
                for (int j = 0; j < 4; j++)
                {
                    int p = tet[j];
                    m_deltas[p] += m_positionDeltasPerConstrain[startIndex + j];
                    m_counts[p]++;
                }
            }
            
        }
    }
}
