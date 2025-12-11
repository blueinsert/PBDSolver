using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.VirtualTexturing;

namespace bluebean.Physics.PBD
{
    public class ParticleCollideConstrainGroup : ConstrainGroup
    {
        public List<ParticleCollisionConstraintsBatch> m_batches = new List<ParticleCollisionConstraintsBatch>();

        public ParticleCollideConstrainGroup(ISolver solver) : base(ConstrainType.ParticleCollide, solver)
        {
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

        public ParticleCollisionConstraintsBatch CreateConstraintsBatch()
        {
            var dataBatch = new ParticleCollisionConstraintsBatch(this);
            m_batches.Add(dataBatch);
            return dataBatch;
        }
    }
}
