using bluebean.Physics.PBD.DataStruct;
using System.Collections;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public class ParticleCollisionConstraintsBatch
    {
        public bool enabled
        {
            set
            {
                if (m_Enabled != value)
                    m_Enabled = value;
            }
            get { return m_Enabled; }
        }

        protected bool m_Enabled = true;

        ParticleCollideConstrainGroup m_owner;
        private ConstrainType m_constrainType;
        public BatchData batchData;

        public ParticleCollisionConstraintsBatch(ParticleCollideConstrainGroup constraintGroup)
        {
            m_owner = constraintGroup;
            m_constrainType = ConstrainType.ParticleCollide;
        }

        public void SetBatchData(BatchData batchData)
        {
            this.batchData = batchData;
        }

        public JobHandle Initialize(JobHandle inputDeps, float substepTime)
        {
            var updateContacts = new UpdateParticleContactsJob()
            {
                prevPositions = m_owner.Solver.PrevParticlePositions,
                velocities = m_owner.Solver.ParticleVels,
                radii = m_owner.Solver.ParticleRadius,
                invMasses = m_owner.Solver.InvMasses,

                contacts = (m_owner.Solver).ParticleContacts,
                batchData = batchData
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return updateContacts.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        public JobHandle Evaluate(JobHandle inputDeps, float stepTime, float substepTime, int substeps)
        {
            var g = m_owner.Solver.Gravity;
            var projectConstraints = new ParticleCollisionConstraintsBatchJob()
            {
                positions = m_owner.Solver.ParticlePositions,
                invMasses = m_owner.Solver.InvMasses,
                radii = m_owner.Solver.ParticleRadius,

                deltas = m_owner.Solver.PositionDeltas,
                counts = m_owner.Solver.PositionConstraintCounts,
                contacts = (m_owner.Solver).ParticleContacts,
                batchData = this.batchData,

                gravity = new float4(g.x, g.y, g.z, 0),
                substepTime = substepTime
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return projectConstraints.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }

        public JobHandle Apply(JobHandle inputDeps, float substepTime)
        {

            var applyConstraints = new ApplyBatchedCollisionConstraintsBatchJob()
            {
                contacts = (m_owner.Solver).ParticleContacts,

                positions = m_owner.Solver.ParticlePositions,
                deltas = m_owner.Solver.PositionDeltas,
                counts = m_owner.Solver.PositionConstraintCounts,

                batchData = batchData,
            };

            int batchCount = batchData.isLast ? batchData.workItemCount : 1;
            return applyConstraints.Schedule(batchData.workItemCount, batchCount, inputDeps);
        }
    }
}