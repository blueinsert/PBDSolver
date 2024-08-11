using bluebean.Physics.PBD.DataStruct;
using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public class CollideConstrainGroup : ConstrainGroup
    {
        public CollideConstrainGroup(ISolver solver) : base(ConstrainType.Collide, solver)
        {

        }

        public override JobHandle Apply(JobHandle inputDeps, float substepTime)
        {
            var projectConstraints = new CollisionConstraintsJob()
            {
                positions = m_solver.ParticlePositions,
                prevPositions = m_solver.PrevParticlePositions,
                //orientations = solverImplementation.orientations,
                //prevOrientations = solverImplementation.prevOrientations,
                invMasses = m_solver.InvMasses,
                radii = m_solver.ParticleRadius,
                //particleMaterialIndices = solverImplementation.collisionMaterials,

                //simplices = solverImplementation.simplices,
                //simplexCounts = solverImplementation.simplexCounts,

                shapes = m_solver.ColliderWorld.m_colliderShapes.AsNativeArray<BurstColliderShape>(),
                transforms = m_solver.ColliderWorld.m_colliderTransforms.AsNativeArray<BurstAffineTransform>(),
                //collisionMaterials = ObiColliderWorld.GetInstance().collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),
                //rigidbodies = ObiColliderWorld.GetInstance().rigidbodies.AsNativeArray<BurstRigidbody>(),
                //rigidbodyLinearDeltas = solverImplementation.abstraction.rigidbodyLinearDeltas.AsNativeArray<float4>(),
                //rigidbodyAngularDeltas = solverImplementation.abstraction.rigidbodyAngularDeltas.AsNativeArray<float4>(),

                deltas = m_solver.PositionDeltas,
                counts = m_solver.PositionConstraintCounts,

                contacts = m_solver.ColliderContacts,
                //inertialFrame = ((BurstSolverImpl)constraints.solver).inertialFrame,
                //constraintParameters = parameters,
                //solverParameters = solverAbstraction.parameters,
                substeps = substeps,
                stepTime = stepTime,
                substepTime = substepTime
            };
            return projectConstraints.Schedule(inputDeps);
        }

        public override JobHandle Solve(JobHandle inputDeps, float substepTime)
        {
            var projectConstraints = new CollisionConstraintsJob()
            {
                positions = m_solver.ParticlePositions,
                prevPositions = m_solver.PrevParticlePositions,
                //orientations = solverImplementation.orientations,
                //prevOrientations = solverImplementation.prevOrientations,
                invMasses = m_solver.InvMasses,
                radii = m_solver.ParticleRadius,
                //particleMaterialIndices = solverImplementation.collisionMaterials,

                //simplices = solverImplementation.simplices,
                //simplexCounts = solverImplementation.simplexCounts,

                shapes = m_solver.ColliderWorld.m_colliderShapes.AsNativeArray<BurstColliderShape>(),
                transforms = m_solver.ColliderWorld.m_colliderTransforms.AsNativeArray<BurstAffineTransform>(),
                //collisionMaterials = ObiColliderWorld.GetInstance().collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),
                //rigidbodies = ObiColliderWorld.GetInstance().rigidbodies.AsNativeArray<BurstRigidbody>(),
                //rigidbodyLinearDeltas = solverImplementation.abstraction.rigidbodyLinearDeltas.AsNativeArray<float4>(),
                //rigidbodyAngularDeltas = solverImplementation.abstraction.rigidbodyAngularDeltas.AsNativeArray<float4>(),

                deltas = m_solver.PositionDeltas,
                counts = m_solver.PositionConstraintCounts,

                contacts = m_solver.ColliderContacts,
                //inertialFrame = ((BurstSolverImpl)constraints.solver).inertialFrame,
                //constraintParameters = parameters,
                //solverParameters = solverAbstraction.parameters,
                substeps = substeps,
                stepTime = stepTime,
                substepTime = substepTime
            };
            return projectConstraints.Schedule(inputDeps);
        }
    }
}
