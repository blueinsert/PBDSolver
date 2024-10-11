
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
        public BurstCellSpan m_oldSpan;
        public BurstCellSpan m_newSpan;
        public int m_entity;
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
        /// <summary>
        /// ǰ�˶�������
        /// </summary>
        [NonSerialized] public List<ColliderHandle> m_colliderHandles;
        [NonSerialized] public NativeColliderShapeList m_colliderShapes;         // list of collider shapes.
        [NonSerialized] public NativeAabbList m_colliderAabbs;                   // list of collider bounds.
        [NonSerialized] public NativeAffineTransformList m_colliderTransforms;   // list of collider transforms.
        /// <summary>
        /// ��ײ������������������������shapeTypeΪTriangleMesh
        /// </summary>
        [NonSerialized] public TriangleMeshContainer m_triangleMeshContainer;
        /// <summary>
        /// ÿ����ײ�������������Ͽ���ķ�Χ
        /// </summary>
        [NonSerialized] public NativeCellSpanList m_colliderCellSpans;

        private int m_colliderCount = 0;
        #endregion

        #region ��ײ�����񻮷֣��ռ��Ż�
        private NativeQueue<MovingCollider> m_movingColliders;
        private NativeMultilevelGrid<int> m_grid;
        #endregion

        /// <summary>
        /// ��ײ�Ӵ�
        /// </summary>
        public NativeQueue<BurstContact> m_colliderContactQueue;

        #endregion

        public void Initialzie(ISolver solver)
        {
            m_solver = solver;

            m_colliderHandles = new List<ColliderHandle>();
            m_colliderShapes = new NativeColliderShapeList();
            m_colliderAabbs = new NativeAabbList();
            m_colliderTransforms = new NativeAffineTransformList();
            m_colliderCellSpans = new NativeCellSpanList();
            m_triangleMeshContainer = new TriangleMeshContainer();
            m_colliderCount = 0;

            m_movingColliders = new NativeQueue<MovingCollider>(Allocator.Persistent);
            m_grid = new NativeMultilevelGrid<int>(1000, Allocator.Persistent);
            m_colliderContactQueue = new NativeQueue<BurstContact>(Allocator.Persistent);
        }

        public void Destroy()
        {
            m_colliderHandles.Clear(); m_colliderHandles = null;
            m_colliderShapes.Dispose(); m_colliderShapes = null;
            m_colliderAabbs.Dispose(); m_colliderAabbs = null;
            m_colliderTransforms.Dispose(); m_colliderTransforms = null;
            m_colliderCellSpans.Dispose(); m_colliderCellSpans = null;
            m_triangleMeshContainer.Dispose(); m_triangleMeshContainer = null;
            m_colliderCount = 0;
            m_movingColliders.Dispose();
            m_grid.Dispose();
            m_colliderContactQueue.Dispose();
        }

        public ColliderHandle CreateCollider()
        {
            var handle = new ColliderHandle(m_colliderHandles.Count);
            m_colliderHandles.Add(handle);

            m_colliderShapes.Add(new ColliderShape() {  });
            m_colliderAabbs.Add(new Aabb());
            m_colliderTransforms.Add(new AffineTransform());
            m_colliderCellSpans.Add(new CellSpan());

            m_colliderCount++;

            return handle;
        }

        public TriangleMeshHandle GetOrCreateTriangleMesh(Mesh mesh)
        {
            return m_triangleMeshContainer.GetOrCreateTriangleMesh(mesh);
        }

        /// <summary>
        /// ������ײ������
        /// ��ǰ������ͬ����ײ��shape,transform,aabb������
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
                //����
                shapes = this.m_colliderShapes.AsNativeArray<BurstColliderShape>(m_colliderCellSpans.count),
                //rigidbodies = world.rigidbodies.AsNativeArray<BurstRigidbody>(),
                //collisionMaterials = world.collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),
                bounds = this.m_colliderAabbs.AsNativeArray<BurstAabb>(m_colliderCellSpans.count),
                //colliderCount = m_colliderCount,
                dt = deltaTime,
                //���
                movingColliders = this.m_movingColliders.AsParallelWriter(),
                colliderCellSpans = this.m_colliderCellSpans.AsNativeArray<BurstCellSpan>(),
            };
            JobHandle movingHandle = identifyMoving.Schedule(m_colliderCount, 128);
            //����multiGrid
            var updateMoving = new UpdateMultiGridByMovingCollidersJob
            {
                movingColliders = m_movingColliders,
                grid = m_grid,
                colliderCount = m_colliderCount
            };
            updateMoving.Schedule(movingHandle).Complete();
        }

        public void UpdateWorld(float deltaTime)
        {
            UpdateColliders();
            UpdateCollidersMultiGrid(deltaTime);
        }

        /// <summary>
        /// ��ײ��⣬�����Ӵ�
        /// </summary>
        /// <param name="deltaTime"></param>
        /// <param name="inputDeps"></param>
        /// <returns></returns>
        public JobHandle GenerateContacts(float deltaTime, JobHandle inputDeps)
        {

            var generateColliderContactsJob = new GenerateContactsJob
            {
                //����
                colliderGrid = m_grid,
                gridLevels = m_grid.populatedLevels.GetKeyArray(Allocator.TempJob),

                positions = Solver.ParticlePositions,
                //orientations = solver.orientations,
                velocities = Solver.ParticleVels,
                invMasses = Solver.InvMasses,
                radii = Solver.ParticleRadius,
                //filters = solver.filters,

                //simplices = solver.simplices,
                //simplexCounts = solver.simplexCounts,
                particleBounds = Solver.ParticleAabb,

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

                
                //solverToWorld = solver.solverToWorld,
                //worldToSolver = solver.worldToSolver,
                deltaTime = deltaTime,
                //parameters = solver.abstraction.parameters
                //���
                contactsQueue = this.m_colliderContactQueue.AsParallelWriter(),
            };

            return generateColliderContactsJob.Schedule(Solver.ParticlePositions.Count(), 16, inputDeps);

        }
    }
}
