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

        private NativeIntList particleIndexList = new NativeIntList();
        private NativeIntList firstIndexList = new NativeIntList();
        private NativeIntList numIndexList = new NativeIntList();
        private NativeFloatList shapeMaterialParameterList = new NativeFloatList();
        private NativeVector4List restComList = new NativeVector4List();
        private NativeVector4List comList = new NativeVector4List();
        private NativeQuaternionList constraintOrientationList = new NativeQuaternionList();
        private NativeMatrix4x4List aqqList = new NativeMatrix4x4List();
        private NativeMatrix4x4List linearTransformList = new NativeMatrix4x4List();
        private NativeMatrix4x4List plasticDeformationList = new NativeMatrix4x4List();

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

        public void AddConstrain(List<int> particles)
        {
            firstIndexList.Add(particleIndexList.count);
            particleIndexList.AddRange(particles);
            numIndexList.Add(particles.Count);
            shapeMaterialParameterList.Add(1f);
            shapeMaterialParameterList.Add(0);
            shapeMaterialParameterList.Add(0);
            shapeMaterialParameterList.Add(0);
            shapeMaterialParameterList.Add(0);
            restComList.Add(new Vector4(0,0,0,0));
            comList.Add(new Vector4(0, 0, 0, 0));
            constraintOrientationList.Add(Quaternion.identity);
            aqqList.Add(Matrix4x4.identity);
            linearTransformList.Add(Matrix4x4.identity);
            plasticDeformationList.Add(Matrix4x4.identity);
            m_ConstraintCount++;

            OnConstrainCountChanged();
        }

        private void OnConstrainCountChanged()
        {
            particleIndices = particleIndexList.AsNativeArray<int>();
            firstIndex = firstIndexList.AsNativeArray<int>();
            numIndices = numIndexList.AsNativeArray<int>();
            shapeMaterialParameters = shapeMaterialParameterList.AsNativeArray<float>();
            restComs = restComList.AsNativeArray<float4>();
            coms = comList.AsNativeArray<float4>();
            constraintOrientations = constraintOrientationList.AsNativeArray<quaternion>();
            Aqq = aqqList.AsNativeArray<float4x4>();
            linearTransforms = linearTransformList.AsNativeArray<float4x4>();
            plasticDeformations = plasticDeformationList.AsNativeArray<float4x4>();
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
                restPositions = m_owner.Solver.ParticleRestPositions,
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


        public void CalculateRestShapeMatching()
        {

            var calculateRest = new ShapeMatchingCalculateRestJob()
            {
                particleIndices = particleIndices,
                firstIndex = firstIndex,
                numIndices = numIndices,
                restComs = restComs,
                Aqq = Aqq,
                deformation = plasticDeformations,

                restPositions = m_owner.Solver.ParticleRestPositions,
                invMasses = m_owner.Solver.InvMasses,
            };

            calculateRest.Schedule(numIndices.Length, 64).Complete();
            Debug.Log($"CalculateRestShapeMatching com:{restComs[0]}");
        }
    }
}