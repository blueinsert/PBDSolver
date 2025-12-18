using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
namespace bluebean.Physics.PBD
{
    public class ShapeMatchingConstrainGroup : ConstrainGroup
    {

        public List<ShapeMatchingContrainsBatch> m_batches = new List<ShapeMatchingContrainsBatch>();

        public ShapeMatchingConstrainGroup(ISolver solver) : base(ConstrainType.ShapeMatching, solver)
        {
        }

        public void AddConstrain(List<int> particles)
        {
            if(m_batches.Count == 0)
            {
                CreateConstraintsBatch();
                m_batches[0].enabled = true;
            }
            var batch = m_batches[0];
            batch.AddConstrain(particles);
            batch.CalculateRestShapeMatching();
        }

        public ShapeMatchingContrainsBatch CreateConstraintsBatch()
        {
            var dataBatch = new ShapeMatchingContrainsBatch(this);
            m_batches.Add(dataBatch);
            return dataBatch;
        }

        public override JobHandle Initialize(JobHandle inputDeps, float substepTime)
        {
            // initialize all batches in parallel:
            if (m_batches.Count > 0)
            {
                NativeArray<JobHandle> deps = new NativeArray<JobHandle>(m_batches.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                for (int i = 0; i < m_batches.Count; ++i)
                    deps[i] = m_batches[i].enabled ? m_batches[i].Initialize(inputDeps, substepTime) : inputDeps;

                JobHandle result = JobHandle.CombineDependencies(deps);
                deps.Dispose();

                return result;
            }

            return inputDeps;
        }

        public override JobHandle Apply(JobHandle inputDeps, float substepTime)
        {
            for (int i = 0; i < m_batches.Count; ++i)
                if (m_batches[i].enabled)
                {
                    inputDeps = m_batches[i].Apply(inputDeps, substepTime);
                    Solver.ScheduleBatchedJobsIfNeeded();
                }
            return inputDeps;
        }

        public override JobHandle Solve(JobHandle inputDeps, float stepTime, float substepTime, int substeps)
        {
            // evaluate all batches:
            for (int i = 0; i < m_batches.Count; ++i)
                if (m_batches[i].enabled)
                {
                    inputDeps = m_batches[i].Evaluate(inputDeps, stepTime, substepTime, substeps);
                    Solver.ScheduleBatchedJobsIfNeeded();
                }
            return inputDeps;
        }
    }
}
