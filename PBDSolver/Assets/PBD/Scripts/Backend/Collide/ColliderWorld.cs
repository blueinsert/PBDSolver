
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

        #region �ڲ�����
        ISolver m_solver;

        #region ��ײ������
        [NonSerialized] public List<ColliderHandle> m_colliderHandles;
        [NonSerialized] public NativeColliderShapeList m_colliderShapes;         // list of collider shapes.
        [NonSerialized] public NativeAabbList m_colliderAabbs;                   // list of collider bounds.
        [NonSerialized] public NativeAffineTransformList m_colliderTransforms;   // list of collider transforms.
        [NonSerialized] public TriangleMeshContainer m_triangleMeshContainer;
        private int m_colliderCount = 0;
        #endregion

        #region ��ײ�����񻮷֣��ռ��Ż�
        private NativeQueue<MovingCollider> movingColliders;
        private NativeMultilevelGrid<int> grid;
        /// <summary>
        /// ÿ����ײ�������������Ͽ���ķ�Χ
        /// </summary>
        public NativeCellSpanList cellSpans;
        #endregion

        public NativeQueue<BurstContact> colliderContactQueue;

        #endregion

        public void Initialzie(ISolver solver)
        {
            m_solver = solver;

            m_colliderHandles = new List<ColliderHandle>();
            m_colliderShapes = new NativeColliderShapeList();
            m_colliderAabbs = new NativeAabbList();
            m_colliderTransforms = new NativeAffineTransformList();
            cellSpans = new NativeCellSpanList();
            m_triangleMeshContainer = new TriangleMeshContainer();
            m_colliderCount = 0;

            movingColliders = new NativeQueue<MovingCollider>(Allocator.Persistent);
            grid = new NativeMultilevelGrid<int>(1000, Allocator.Persistent);
            colliderContactQueue = new NativeQueue<BurstContact>(Allocator.Persistent);
        }

        public void Destroy()
        {
            m_colliderHandles.Clear(); m_colliderHandles = null;
            m_colliderShapes.Dispose(); m_colliderShapes = null;
            m_colliderAabbs.Dispose(); m_colliderAabbs = null;
            m_colliderTransforms.Dispose(); m_colliderTransforms = null;
            cellSpans.Dispose(); cellSpans = null;
            m_triangleMeshContainer.Dispose(); m_triangleMeshContainer = null;
            m_colliderCount = 0;
            movingColliders.Dispose();
            grid.Dispose();
            colliderContactQueue.Dispose();
        }

        public ColliderHandle CreateCollider()
        {
            var handle = new ColliderHandle(m_colliderHandles.Count);
            m_colliderHandles.Add(handle);

            m_colliderShapes.Add(new ColliderShape() {  });
            m_colliderAabbs.Add(new Aabb());
            m_colliderTransforms.Add(new AffineTransform());

            cellSpans.Add(new CellSpan());
            m_colliderCount++;

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
            if (m_colliderCount <= 0)
                return;
            //��ȡ�ƶ�����ײ����䷶Χ
            var identifyMoving = new IdentifyMovingCollidersJob
            {
                movingColliders = this.movingColliders.AsParallelWriter(),
                shapes = this.m_colliderShapes.AsNativeArray<BurstColliderShape>(cellSpans.count),
                //rigidbodies = world.rigidbodies.AsNativeArray<BurstRigidbody>(),
                //collisionMaterials = world.collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),
                bounds = this.m_colliderAabbs.AsNativeArray<BurstAabb>(cellSpans.count),
                cellIndices = this.cellSpans.AsNativeArray<BurstCellSpan>(),
                colliderCount = m_colliderCount,
                dt = deltaTime
            };
            JobHandle movingHandle = identifyMoving.Schedule(m_colliderCount, 128);
            //����multiGrid
            var updateMoving = new UpdateMovingCollidersJob
            {
                movingColliders = movingColliders,
                grid = grid,
                colliderCount = m_colliderCount
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

        public void UpdateWorld(float deltaTime) {
            UpdateColliders();
            UpdateCollidersMultiGrid(deltaTime);
        }
 
    }
}
