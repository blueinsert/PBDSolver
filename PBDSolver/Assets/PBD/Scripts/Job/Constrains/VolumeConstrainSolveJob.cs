using bluebean.Physics.PBD.DataStruct;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct VolumeConstrainSolveJob : IJobParallelFor
    {
        /// <summary>
        /// �����嶥������
        /// </summary>
        [ReadOnly] public NativeArray<int4> m_tets;
        /// <summary>
        /// �������ʼ���
        /// </summary>
        [ReadOnly] public NativeArray<float> m_restVolumes;
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

        /// <summary>
        /// ����Լ����������λ���ݶ�
        /// </summary>
        [NativeDisableContainerSafetyRestriction]
        [NativeDisableParallelForRestriction]
        public NativeArray<float4> m_gradientsPerConstrain;


        public void Execute(int index)
        {
            //index ��Լ������������������
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

            m_gradientsPerConstrain[startIndex] = new float4(math.cross((p4 - p2).xyz, (p3 - p2).xyz), 0);
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
