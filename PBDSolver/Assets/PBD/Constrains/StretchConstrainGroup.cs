using bluebean.UGFramework.DataStruct;
using bluebean.UGFramework.DataStruct.Native;
using System.Numerics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.UGFramework.Physics
{
    public class StretchConstrainGroup : ConstrainGroup
    {
        /// <summary>
        /// 边顶点索引数组
        /// </summary>
        public NativeInt2List m_edgeList = new NativeInt2List();
        /// <summary>
        /// 每个约束(边)的初始长度
        /// </summary>
        public NativeFloatList m_restLenList = new NativeFloatList();
        /// <summary>
        /// 约束柔度
        /// </summary>
        public NativeFloatList m_complianceList = new NativeFloatList();
        public NativeVector4List m_positionDeltaPerConstrainList = new NativeVector4List();

        public NativeArray<float4> m_positionDeltasPerConstrain;
        private NativeArray<int2> m_edges;
        private NativeArray<float> m_restLens;
        private NativeArray<float> m_compliances;

        public StretchConstrainGroup(ISolver solver) : base(ConstrainType.Stretch, solver)
        {
            
        }

        private void OnConstrainCountChanged()
        {
            m_edges = m_edgeList.AsNativeArray<int2>();
            m_restLens = m_restLenList.AsNativeArray<float>();
            m_compliances = m_complianceList.AsNativeArray<float>();
            m_positionDeltasPerConstrain = m_positionDeltaPerConstrainList.AsNativeArray<float4>();
        }

        public void AddConstrain(VectorInt2 edge, float restLen,float compliance)
        {
            m_edgeList.Add(edge);
            m_restLenList.Add(restLen);
            m_complianceList.Add(compliance);
            m_positionDeltaPerConstrainList.Add(UnityEngine.Vector4.zero);
            m_positionDeltaPerConstrainList.Add(UnityEngine.Vector4.zero);
            m_constrainCount++;

            OnConstrainCountChanged();
        }

        public override JobHandle Apply(JobHandle inputDeps, float substepTime)
        {
            /*
            var sumJob = new StretchConstrainSummarizeJob()
            {
                m_edges = this.m_edges,
                m_positionDeltasPerConstrain = this.m_positionDeltasPerConstrain,
                m_deltas = this.m_solver.PositionDeltas,
                m_counts = this.m_solver.PositionConstraintCounts,
            };
            */
            //inputDeps = sumJob.Schedule(m_constrainCount, 32, inputDeps);

            var sumJobSe = new StretchConstrainSummarizeJobSequential(){
                m_edges = this.m_edges,
                m_positionDeltasPerConstrain = this.m_positionDeltasPerConstrain,
                m_deltas = this.m_solver.PositionDeltas,
                m_counts = this.m_solver.PositionConstraintCounts,
            };
            //sumJobSe.Schedule(inputDeps).Complete();
            inputDeps = sumJobSe.Schedule(inputDeps);
            var job = new PositionDeltaApplyJob() {
                m_positions = this.m_solver.ParticlePositions,
                m_deltas = this.m_solver.PositionDeltas,
                m_counts = this.m_solver.PositionConstraintCounts,
                m_particleProperties = this.m_solver.ParticleProperties,
            };
            return job.Schedule(this.m_solver.ParticlePositions.Length, 32, inputDeps);
        }

        public override JobHandle Solve(JobHandle inputDeps, float substepTime)
        {
            var job = new StretchConstrainSolveJob() {
                m_edges = this.m_edges,
                m_restLen = this.m_restLens,
                m_compliances = this.m_compliances,
                m_invMasses = this.m_solver.InvMasses,
                m_positions = this.m_solver.ParticlePositions,
                m_deltaTimeSqr = substepTime*substepTime,

                m_positionDeltasPerConstrain = this.m_positionDeltasPerConstrain,
                m_compliance = this.m_solver.StretchConstrainCompliance,
            };
            return job.Schedule(m_constrainCount, 32, inputDeps);
        }
    }
}
