using bluebean.UGFramework.DataStruct;
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
    [BurstCompile]
    public struct VolumeConstrainSolveJob : IJobParallelFor
    {
        /// <summary>
        /// 四面体顶点数组
        /// </summary>
        [ReadOnly] public NativeArray<int4> m_tets;
        /// <summary>
        /// 四面体初始体积
        /// </summary>
        [ReadOnly] public NativeArray<float> m_restVolumes;
        /// <summary>
        /// 约束柔度
        /// </summary>
        [ReadOnly] public NativeArray<float> m_compliances;
        [ReadOnly] public float m_compliance;
        /// <summary>
        /// 粒子位置数组
        /// </summary>
        [ReadOnly] public NativeArray<float4> m_positions;
        /// <summary>
        /// 粒子质量
        /// </summary>
        [ReadOnly] public NativeArray<float> m_invMasses;
        [ReadOnly] public float m_deltaTimeSqr;

        [NativeDisableContainerSafetyRestriction]
        [NativeDisableParallelForRestriction]
        public NativeArray<float4> m_positionDeltasPerConstrain;

        /// <summary>
        /// 本次约束求解产生的位置梯度
        /// </summary>
        [NativeDisableContainerSafetyRestriction]
        [NativeDisableParallelForRestriction] 
        public NativeArray<float4> m_gradientsPerConstrain;

        
        public void Execute(int index)
        {
            //index 是约束索引或四面体索引
            float alpha = m_compliance / m_deltaTimeSqr;
            int startIndex = index * 4;

            var tet = m_tets[index];
            var p1Index = tet.x;
            var p2Index = tet.y;
            var p3Index = tet.z;
            var p4Index = tet.w;
            var p1 = m_positions[p1Index];
            var p2 = m_positions[p2Index];
            var p3 = m_positions[p3Index];
            var p4 = m_positions[p4Index];

            m_gradientsPerConstrain[startIndex] = new float4(math.cross((p4 - p2).xyz, (p3 - p2).xyz),0);
            m_gradientsPerConstrain[startIndex + 1] = new float4(math.cross((p3 - p1).xyz, (p4 - p1).xyz), 0);
            m_gradientsPerConstrain[startIndex + 2] = new float4(math.cross((p4 - p1).xyz, (p2 - p1).xyz), 0);
            m_gradientsPerConstrain[startIndex + 3] = new float4(math.cross((p2 - p1).xyz, (p3 - p1).xyz), 0);
            float w = 0;
            for (int j = 0; j < 4; j++)
            {
                int p = tet[j];
                w += m_invMasses[p] * math.lengthsq(m_gradientsPerConstrain[startIndex + j]);
            }
            var vol = BurstMath.CalcTetVolume(p1.xyz, p2.xyz, p3.xyz, p4.xyz);
            var restVol = m_restVolumes[index];
            float C = (vol - restVol) * 6f;
            float s = -C / (w + alpha);

            for (int j = 0; j < 4; j++)
            {
                var p = tet[j];
                float4 delta = m_gradientsPerConstrain[startIndex + j] * s * m_invMasses[p];
                m_positionDeltasPerConstrain[startIndex + j] = delta;
            }
        }

    }
}
