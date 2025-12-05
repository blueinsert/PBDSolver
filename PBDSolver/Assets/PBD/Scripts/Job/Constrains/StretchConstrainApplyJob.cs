using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    /// <summary>
    /// ����
    /// </summary>
    [BurstCompile]
    public struct StretchConstrainSummarizeJob : IJobParallelFor
    {
        /// <summary>
        /// �߶�����������
        /// </summary>
        [ReadOnly] public NativeArray<int2> m_edges;
        [ReadOnly] public NativeArray<float4> m_positionDeltasPerConstrain;

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
            int constrainIndex = index;

            var e = m_edges[constrainIndex];
            var i = e[0];
            var j = e[1];
            var startIndex = constrainIndex * 2;
            m_counts[i]++;
            m_counts[j]++;
            m_deltas[i] += m_positionDeltasPerConstrain[startIndex];
            m_deltas[j] += m_positionDeltasPerConstrain[startIndex + 1];


        }
    }

    /// <summary>
    /// ����
    /// </summary>
    [BurstCompile]
    public struct StretchConstrainSummarizeJobSequential : IJob
    {
        /// <summary>
        /// �߶�����������
        /// </summary>
        [ReadOnly] public NativeArray<int2> m_edges;
        [ReadOnly] public NativeArray<float4> m_positionDeltasPerConstrain;

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

        public void Execute()
        {
            for (int index = 0; index < m_edges.Length; index++)
            {
                var e = m_edges[index];
                var i = e[0];
                var j = e[1];
                var startIndex = index * 2;
                m_counts[i]++;
                m_counts[j]++;
                m_deltas[i] += m_positionDeltasPerConstrain[startIndex];
                m_deltas[j] += m_positionDeltasPerConstrain[startIndex + 1];
            }



        }
    }
}
