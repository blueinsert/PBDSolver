
using bluebean.Physics.PBD.DataStruct;
using bluebean.Physics.PBD.DataStruct.Native;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
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

        #region 内部变量
        ISolver m_solver;

        #region 碰撞体数据
        [NonSerialized] public List<ColliderHandle> m_colliderHandles;
        [NonSerialized] public NativeColliderShapeList m_colliderShapes;         // list of collider shapes.
        [NonSerialized] public NativeAabbList m_colliderAabbs;                   // list of collider bounds.
        [NonSerialized] public NativeAffineTransformList m_colliderTransforms;   // list of collider transforms.
        [NonSerialized] public TriangleMeshContainer m_triangleMeshContainer;
        private int colliderCount = 0;
        #endregion

        #region 碰撞体网格划分，空间优化
        private NativeQueue<MovingCollider> movingColliders;
        private NativeMultilevelGrid<int> grid;
        public NativeCellSpanList cellSpans;
        #endregion

        public NativeQueue<BurstContact> colliderContactQueue;

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
        /// 更新碰撞体数据
        /// </summary>
        public void UpdateColliders()
        {
            // update all colliders:
            for (int i = 0; i < m_colliderHandles.Count; ++i)
                m_colliderHandles[i].owner.UpdateIfNeeded();
        }

        /// <summary>
        /// 更新网格划分
        /// </summary>
        public void UpdateCollidersMultiGrid(float deltaTime)
        {
            //将移动的碰撞体加入队列
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

            var generateColliderContactsJob = new GenerateContactsJob
            {
                colliderGrid = grid,
                gridLevels = grid.populatedLevels.GetKeyArray(Allocator.TempJob),

                positions = Solver.ParticlePositions,
                //orientations = solver.orientations,
                velocities = Solver.ParticleVels,
                invMasses = Solver.InvMasses,
                radii = Solver.ParticleRadius,
                //filters = solver.filters,

                //simplices = solver.simplices,
                //simplexCounts = solver.simplexCounts,
                simplexBounds = Solver.ParticleAabb,

                transforms = this.m_colliderTransforms.AsNativeArray<BurstAffineTransform>(),
                shapes = this.m_colliderShapes.AsNativeArray<BurstColliderShape>(),
                //rigidbodies = world.rigidbodies.AsNativeArray<BurstRigidbody>(),
                //collisionMaterials = world.collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),
                bounds = this.m_colliderAabbs.AsNativeArray<BurstAabb>(),

                //distanceFieldHeaders = world.distanceFieldContainer.headers.AsNativeArray<DistanceFieldHeader>(),
                //distanceFieldNodes = world.distanceFieldContainer.dfNodes.AsNativeArray<BurstDFNode>(),

                triangleMeshHeaders = this.m_triangleMeshContainer.headers.AsNativeArray<TriangleMeshHeader>(),
                bihNodes = this.m_triangleMeshContainer.bihNodes.AsNativeArray<BIHNode>(),
                triangles = this.m_triangleMeshContainer.triangles.AsNativeArray<Triangle>(),
                vertices = this.m_triangleMeshContainer.vertices.AsNativeArray<float3>(),

                //edgeMeshHeaders = world.edgeMeshContainer.headers.AsNativeArray<EdgeMeshHeader>(),
                //edgeBihNodes = world.edgeMeshContainer.bihNodes.AsNativeArray<BIHNode>(),
                //edges = world.edgeMeshContainer.edges.AsNativeArray<Edge>(),
                //edgeVertices = world.edgeMeshContainer.vertices.AsNativeArray<float2>(),

                //heightFieldHeaders = world.heightFieldContainer.headers.AsNativeArray<HeightFieldHeader>(),
                //heightFieldSamples = world.heightFieldContainer.samples.AsNativeArray<float>(),

                contactsQueue = this.colliderContactQueue.AsParallelWriter(),
                //solverToWorld = solver.solverToWorld,
                //worldToSolver = solver.worldToSolver,
                deltaTime = deltaTime,
                //parameters = solver.abstraction.parameters
            };

            return generateColliderContactsJob.Schedule(Solver.ParticlePositions.Count(), 16, inputDeps);

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
