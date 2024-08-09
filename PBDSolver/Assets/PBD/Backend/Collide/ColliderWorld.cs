
using bluebean.Physics.PBD.DataStruct;
using bluebean.Physics.PBD.DataStruct.Native;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public struct MovingCollider
    {
        public BurstCellSpan oldSpan;
        public BurstCellSpan newSpan;
        public int entity;
    }

    public class ColliderWorld
    {
        ISolver Solver
        {
            get
            {
                return m_solver;
            }
        }

        #region �ڲ�����
        ISolver m_solver;

        #region ��ײ������
        [NonSerialized] public List<ColliderHandle> m_colliderHandles;
        [NonSerialized] public NativeColliderShapeList m_colliderShapes;         // list of collider shapes.
        [NonSerialized] public NativeAabbList m_colliderAabbs;                   // list of collider bounds.
        [NonSerialized] public NativeAffineTransformList m_colliderTransforms;   // list of collider transforms.
        [NonSerialized] public TriangleMeshContainer m_triangleMeshContainer;
        private int colliderCount = 0;
        #endregion

        #region ��ײ�����񻮷֣��ռ��Ż�
        private NativeQueue<MovingCollider> movingColliders;
        private NativeMultilevelGrid<int> grid;
        public NativeCellSpanList cellSpans;
        #endregion

        public NativeArray<int> simplices;
        public NativeArray<BurstAabb> simplexBounds;
        public NativeArray<float4> principalRadii;

        public NativeArray<BurstContact> colliderContacts;

        #endregion

        public void Initialzie(ISolver solver)
        {
            m_colliderHandles = new List<ColliderHandle>();
            m_colliderShapes = new NativeColliderShapeList();
            m_colliderAabbs = new NativeAabbList();
            m_colliderTransforms = new NativeAffineTransformList();
            m_triangleMeshContainer = new TriangleMeshContainer();
            colliderCount = 0;
        }

        public ColliderHandle CreateCollider()
        {
            var handle = new ColliderHandle(m_colliderHandles.Count);
            m_colliderHandles.Add(handle);

            m_colliderShapes.Add(new ColliderShape() {  });
            m_colliderAabbs.Add(new Aabb());
            m_colliderTransforms.Add(new AffineTransform());

            return handle;
        }

        public TriangleMeshHandle GetOrCreateTriangleMesh(Mesh mesh)
        {
            return m_triangleMeshContainer.GetOrCreateTriangleMesh(mesh);
        }

        /// <summary>
        /// ������ײ������
        /// </summary>
        public void UpdateColliders()
        {
            // update all colliders:
            for (int i = 0; i < m_colliderHandles.Count; ++i)
                m_colliderHandles[i].owner.UpdateIfNeeded();
        }

        /// <summary>
        /// �������񻮷�
        /// </summary>
        public void UpdateCollidersMultiGrid(float deltaTime)
        {
            //���ƶ�����ײ��������
            var identifyMoving = new IdentifyMovingCollidersJob
            {
                movingColliders = this.movingColliders.AsParallelWriter(),
                shapes = this.m_colliderShapes.AsNativeArray<BurstColliderShape>(cellSpans.count),
                //rigidbodies = world.rigidbodies.AsNativeArray<BurstRigidbody>(),
                //collisionMaterials = world.collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),
                bounds = this.m_colliderAabbs.AsNativeArray<BurstAabb>(cellSpans.count),
                cellIndices = this.cellSpans.AsNativeArray<BurstCellSpan>(),
                colliderCount = colliderCount,
                dt = deltaTime
            };
            JobHandle movingHandle = identifyMoving.Schedule(cellSpans.count, 128);

            var updateMoving = new UpdateMovingColliders
            {
                movingColliders = movingColliders,
                grid = grid,
                colliderCount = colliderCount
            };
            updateMoving.Schedule(movingHandle).Complete();
        }

        public JobHandle GenerateContacts(float deltaTime, JobHandle inputDeps)
        {
            var world = ObiColliderWorld.GetInstance();

            var generateColliderContactsJob = new GenerateContactsJob
            {
                colliderGrid = grid,
                gridLevels = grid.populatedLevels.GetKeyArray(Allocator.TempJob),

                positions = Solver.ParticlePositions,
                orientations = solver.orientations,
                velocities = solver.velocities,
                invMasses = solver.invMasses,
                radii = solver.principalRadii,
                filters = solver.filters,

                simplices = solver.simplices,
                simplexCounts = solver.simplexCounts,
                simplexBounds = solver.simplexBounds,

                transforms = world.colliderTransforms.AsNativeArray<BurstAffineTransform>(),
                shapes = world.colliderShapes.AsNativeArray<BurstColliderShape>(),
                rigidbodies = world.rigidbodies.AsNativeArray<BurstRigidbody>(),
                collisionMaterials = world.collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),
                bounds = world.colliderAabbs.AsNativeArray<BurstAabb>(),

                distanceFieldHeaders = world.distanceFieldContainer.headers.AsNativeArray<DistanceFieldHeader>(),
                distanceFieldNodes = world.distanceFieldContainer.dfNodes.AsNativeArray<BurstDFNode>(),

                triangleMeshHeaders = world.triangleMeshContainer.headers.AsNativeArray<TriangleMeshHeader>(),
                bihNodes = world.triangleMeshContainer.bihNodes.AsNativeArray<BIHNode>(),
                triangles = world.triangleMeshContainer.triangles.AsNativeArray<Triangle>(),
                vertices = world.triangleMeshContainer.vertices.AsNativeArray<float3>(),

                edgeMeshHeaders = world.edgeMeshContainer.headers.AsNativeArray<EdgeMeshHeader>(),
                edgeBihNodes = world.edgeMeshContainer.bihNodes.AsNativeArray<BIHNode>(),
                edges = world.edgeMeshContainer.edges.AsNativeArray<Edge>(),
                edgeVertices = world.edgeMeshContainer.vertices.AsNativeArray<float2>(),

                heightFieldHeaders = world.heightFieldContainer.headers.AsNativeArray<HeightFieldHeader>(),
                heightFieldSamples = world.heightFieldContainer.samples.AsNativeArray<float>(),

                contactsQueue = colliderContactQueue.AsParallelWriter(),
                solverToWorld = solver.solverToWorld,
                worldToSolver = solver.worldToSolver,
                deltaTime = deltaTime,
                parameters = solver.abstraction.parameters
            };

            return generateColliderContactsJob.Schedule(solver.simplexCounts.simplexCount, 16, inputDeps);

        }

        public void OnPreSubStep(float dt, Vector3 g)
        {

        }

        public void OnPostSubStep(float dt, float velDamp)
        {

        }

        public void OnPreStep() {
            UpdateColliders();
        }

        public void OnPostStep()
        {
        }
    }
}
