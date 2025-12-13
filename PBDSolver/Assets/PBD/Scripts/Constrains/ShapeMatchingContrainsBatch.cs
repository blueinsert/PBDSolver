using bluebean.Physics.PBD.DataStruct.Native;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public class ShapeMatchingContrainsBatch
    {
        public ShapeMatchingConstrainGroup m_owner;
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

        private int m_ConstraintCount = 0;
        private NativeArray<int> particleIndices;

        private NativeArray<int> firstIndex;
        private NativeArray<int> numIndices;
        private NativeArray<float> shapeMaterialParameters;
        private NativeArray<float4> restComs;
        private NativeArray<float4> coms;
        private NativeArray<quaternion> constraintOrientations;

        private NativeArray<float4x4> Aqq;
        private NativeArray<float4x4> linearTransforms;
        private NativeArray<float4x4> plasticDeformations;

        public ShapeMatchingContrainsBatch(ShapeMatchingConstrainGroup owner)
        {
            m_owner = owner;
        }

        public void SetShapeMatchingConstraints(NativeIntList particleIndices,
                                               NativeIntList firstIndex,
                                               NativeIntList numIndices,
                                               NativeFloatList shapeMaterialParameters,
                                               NativeVector4List restComs,
                                               NativeVector4List coms,
                                               NativeQuaternionList constraintOrientations,
                                               NativeMatrix4x4List linearTransforms,
                                               NativeMatrix4x4List plasticDeformations,
                                               int count)
        {
            this.particleIndices = particleIndices.AsNativeArray<int>();
            this.firstIndex = firstIndex.AsNativeArray<int>();
            this.numIndices = numIndices.AsNativeArray<int>();
            this.shapeMaterialParameters = shapeMaterialParameters.AsNativeArray<float>();
            this.restComs = restComs.AsNativeArray<float4>();
            this.coms = coms.AsNativeArray<float4>();
            this.constraintOrientations = constraintOrientations.AsNativeArray<quaternion>();
            this.linearTransforms = linearTransforms.AsNativeArray<float4x4>();
            this.plasticDeformations = plasticDeformations.AsNativeArray<float4x4>();

            if (Aqq.IsCreated)
                Aqq.Dispose();

            Aqq = new NativeArray<float4x4>(count, Allocator.Persistent);

            m_ConstraintCount = count;
        }

        public  JobHandle Initialize(JobHandle inputDeps, float substepTime)
        {
            return inputDeps;
        }

        public  JobHandle Evaluate(JobHandle inputDeps, float stepTime, float substepTime, int substeps)
        {
            var projectConstraints = new ShapeMatchingConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                firstIndex = firstIndex,
                numIndices = numIndices,
                shapeMaterialParameters = shapeMaterialParameters,
                restComs = restComs,
                coms = coms,
                constraintOrientations = constraintOrientations,
                Aqq = Aqq,
                linearTransforms = linearTransforms,
                deformation = plasticDeformations,

                positions = m_owner.Solver.ParticlePositions,
                restPositions = m_owner.Solver.ParticlePositions,//todo
                invMasses = m_owner.Solver.InvMasses,

                deltas = m_owner.Solver.PositionDeltas,
                counts = m_owner.Solver.PositionConstraintCounts,

                deltaTime = substepTime
            };

            return projectConstraints.Schedule(m_ConstraintCount, 4, inputDeps);
        }

        public JobHandle Apply(JobHandle inputDeps, float substepTime)
        {
            //var parameters = solverAbstraction.GetConstraintParameters(m_ConstraintType);

            var applyConstraints = new ApplyShapeMatchingConstraintsBatchJob()
            {
                particleIndices = particleIndices,
                firstIndex = firstIndex,
                numIndices = numIndices,

                positions = m_owner.Solver.ParticlePositions,
                deltas = m_owner.Solver.PositionDeltas,
                counts = m_owner.Solver.PositionConstraintCounts,

                //sorFactor = parameters.SORFactor
                sorFactor = 1.0f,
            };

            return applyConstraints.Schedule(m_ConstraintCount, 8, inputDeps);
        }
    }
}