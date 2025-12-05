using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct StretchConstrainSolveJob : IJobParallelFor
    {
        /// <summary>
        /// �߶�����������
        /// </summary>
        [ReadOnly] public NativeArray<int2> m_edges;
        /// <summary>
        /// ÿ��Լ��(��)�ĳ�ʼ����
        /// </summary>
        [ReadOnly] public NativeArray<float> m_restLen;
        /// <summary>
        /// Լ�����
        /// </summary>
        [ReadOnly] public NativeArray<float> m_compliances;
        [ReadOnly] public float m_compliance;
        /// <summary>
        /// ����λ������
        /// </summary>
        [ReadOnly] public NativeArray<float4> m_positions;
        /// <summary>
        /// ��������
        /// </summary>
        [ReadOnly] public NativeArray<float> m_invMasses;

        [ReadOnly] public float m_deltaTimeSqr;

        [NativeDisableContainerSafetyRestriction]
        [NativeDisableParallelForRestriction]
        public NativeArray<float4> m_positionDeltasPerConstrain;

        public void Execute(int index)
        {
            float alpha = m_compliance / m_deltaTimeSqr;

            var l_e = m_restLen[index];
            var e = m_edges[index];
            var i = e[0];
            var j = e[1];
            var x_i = m_positions[i];
            var x_j = m_positions[j];
            var x_ij = x_j - x_i;
            var dir = math.normalize(x_ij);
            var grads = dir;
            var len = math.length(x_ij);
            {
                //xpdb
                var inv_mass_i = m_invMasses[i];
                var inv_mass_j = m_invMasses[j];
                float C = len - l_e;
                float w = inv_mass_i + inv_mass_j;
                var s = -C / (w + alpha);
                var delta1 = -grads * s * inv_mass_i;
                var delta2 = grads * s * inv_mass_j;

                var startIndex = index * 2;
                m_positionDeltasPerConstrain[startIndex] = delta1;
                m_positionDeltasPerConstrain[startIndex + 1] = delta2;

            }
        }
    }
}
