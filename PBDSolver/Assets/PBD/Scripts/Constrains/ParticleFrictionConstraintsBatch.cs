using bluebean.Physics.PBD.DataStruct;
using System.Collections;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public class ParticleFrictionConstraintsBatch
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

        ParticleFrictionConstrainGroup m_owner;
        private ConstrainType m_constrainType;
        public BatchData batchData;

        public ParticleFrictionConstraintsBatch(ParticleFrictionConstrainGroup constraintGroup)
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
            return inputDeps;
        }

        public JobHandle Evaluate(JobHandle inputDeps, float stepTime, float substepTime, int substeps)
        {
            var g = m_owner.Solver.Gravity;
            var projectConstraints = new ParticleFrictionConstraintsBatchJob()
            {
                positions = m_owner.Solver.ParticlePositions,
                prevPositions = m_owner.Solver.PrevParticlePositions,
                invMasses = m_owner.Solver.InvMasses,
                radii = m_owner.Solver.ParticleRadius,
                staticFrictions = m_owner.Solver.StaticFriction,
                dynamicFrictions = m_owner.Solver.DynamicFriction,

                deltas = m_owner.Solver.PositionDeltas,
                counts = m_owner.Solver.PositionConstraintCounts,
                contacts = (m_owner.Solver).ParticleContacts,

                batchData = this.batchData,
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