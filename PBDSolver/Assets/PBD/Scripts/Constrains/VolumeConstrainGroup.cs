using bluebean.Physics.PBD.DataStruct;
using bluebean.Physics.PBD.DataStruct.Native;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    public class VolumeConstrainGroup : ConstrainGroup
    {
        /// <summary>
        /// �����嶥����������
        /// </summary>
        public NativeInt4List m_tetList = new NativeInt4List();
        /// <summary>
        /// ÿ��Լ��(������)�ĳ�ʼ���
        /// </summary>
        public NativeFloatList m_restVolumeList = new NativeFloatList();
        /// <summary>
        /// Լ�����
        /// </summary>
        public NativeFloatList m_complianceList = new NativeFloatList();

        public NativeVector4List m_positionDeltaPerConstrainList = new NativeVector4List();
        public NativeVector4List m_positionGradientPerConstrainList = new NativeVector4List();

        private NativeArray<int4> m_tets;
        private NativeArray<float> m_restVolumes;
        private NativeArray<float> m_compliances;
        public NativeArray<float4> m_positionDeltasPerConstrain;
        public NativeArray<float4> m_positionGradientsPerConstrain;

        public VolumeConstrainGroup(ISolver solver) : base(ConstrainType.Volume, solver)
        {

        }

        private void OnConstrainCountChanged()
        {
            m_tets = m_tetList.AsNativeArray<int4>();
            m_restVolumes = m_restVolumeList.AsNativeArray<float>();
            m_compliances = m_complianceList.AsNativeArray<float>();
            m_positionDeltasPerConstrain = m_positionDeltaPerConstrainList.AsNativeArray<float4>();
            m_positionGradientsPerConstrain = m_positionGradientPerConstrainList.AsNativeArray<float4>();
        }

        public void AddConstrain(VectorInt4 tet, float restVolume, float compliance)
        {
            m_tetList.Add(tet);
            m_restVolumeList.Add(restVolume);
            m_complianceList.Add(compliance);
            m_positionDeltaPerConstrainList.Add(UnityEngine.Vector4.zero);
            m_positionDeltaPerConstrainList.Add(UnityEngine.Vector4.zero);
            m_positionDeltaPerConstrainList.Add(UnityEngine.Vector4.zero);
            m_positionDeltaPerConstrainList.Add(UnityEngine.Vector4.zero);
            m_positionGradientPerConstrainList.Add(UnityEngine.Vector4.zero);
            m_positionGradientPerConstrainList.Add(UnityEngine.Vector4.zero);
            m_positionGradientPerConstrainList.Add(UnityEngine.Vector4.zero);
            m_positionGradientPerConstrainList.Add(UnityEngine.Vector4.zero);
            m_constrainCount++;

            OnConstrainCountChanged();
        }

        public override JobHandle Apply(JobHandle inputDeps, float substepTime)
        {
            /*
            var sumJob = new VolumeConstrainSummarizeJob()
            {
                m_tets = this.m_tets,
                m_positionDeltasPerConstrain = this.m_positionDeltasPerConstrain,
                m_deltas = this.m_solver.PositionDeltas,
                m_counts = this.m_solver.PositionConstraintCounts,
            };
            inputDeps = sumJob.Schedule(m_constrainCount, 32, inputDeps);
            */

            var sumJobSe = new VolumeConstrainSummarizeJobSequential()
            {
                m_tets = this.m_tets,
                m_positionDeltasPerConstrain = this.m_positionDeltasPerConstrain,
                m_deltas = this.m_solver.PositionDeltas,
                m_counts = this.m_solver.PositionConstraintCounts,
            };
            inputDeps = sumJobSe.Schedule(inputDeps);

            var job = new PositionDeltaApplyJob()
            {
                m_positions = this.m_solver.ParticlePositions,
                m_deltas = this.m_solver.PositionDeltas,
                m_counts = this.m_solver.PositionConstraintCounts,
                m_particleProperties = this.m_solver.ParticleProperties,
            };
            return job.Schedule(this.m_solver.ParticlePositions.Length, 32, inputDeps);
        }

        public override JobHandle Solve(JobHandle inputDeps, float stepTime, float substepTime, int substeps)
        {
            var job = new VolumeConstrainSolveJob()
            {
                m_tets = this.m_tets,
                m_restVolumes = this.m_restVolumes,
                m_compliances = this.m_compliances,
                m_compliance = this.m_solver.VolumeConstrainCompliance,
                m_invMasses = this.m_solver.InvMasses,
                m_positions = this.m_solver.ParticlePositions,
                m_gradientsPerConstrain = this.m_positionGradientsPerConstrain,
                m_positionDeltasPerConstrain = this.m_positionDeltasPerConstrain,
                m_deltaTimeSqr = substepTime * substepTime,
            };
            return job.Schedule(m_constrainCount, 32, inputDeps);
        }
    }
}
