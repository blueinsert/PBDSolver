using bluebean.Physics.PBD.DataStruct;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    public class FrictionConstrainGroup : ConstrainGroup
    {
        public FrictionConstrainGroup(ISolver solver) : base(ConstrainType.Friction, solver)
        {

        }

        public override JobHandle Apply(JobHandle inputDeps, float substepTime)
        {
            var applyConstraints = new ApplyCollisionConstraintsJob()
            {
                contacts = m_solver.ColliderContacts,

                positions = m_solver.ParticlePositions,
                deltas = m_solver.PositionDeltas,
                counts = m_solver.PositionConstraintCounts,
            };

            return applyConstraints.Schedule(inputDeps);
        }

        public override JobHandle Solve(JobHandle inputDeps, float stepTime, float substepTime, int substeps)
        {
             FrictionConstraintsBatchJob job = new FrictionConstraintsBatchJob() {
                 positions = m_solver.ParticlePositions,
                 prevPositions = m_solver.PrevParticlePositions,
                 invMasses = m_solver.InvMasses,
                 radii = m_solver.ParticleRadius,
                 staticFrictions = m_solver.StaticFriction,
                 dynamicFrictions = m_solver.DynamicFriction,

                 shapes = m_solver.ColliderWorld.m_colliderShapes.AsNativeArray<BurstColliderShape>(),
                 transforms = m_solver.ColliderWorld.m_colliderTransforms.AsNativeArray<BurstAffineTransform>(),
                 rigidbodies = m_solver.ColliderWorld.m_rigidbodies.AsNativeArray<BurstRigidbody>(),
                 rigidbodyLinearDeltas = m_solver.RigidbodyLinearDeltas.AsNativeArray<float4>(),
                 rigidbodyAngularDeltas = m_solver.RigidbodyAngularDeltas.AsNativeArray<float4>(),

                 deltas = m_solver.PositionDeltas,
                 counts = m_solver.PositionConstraintCounts,

                 contacts = m_solver.ColliderContacts,

                 substeps = substeps,
                 stepTime = stepTime,
                 substepTime = substepTime
             };

            return job.Schedule(inputDeps);
        }
    }
}
