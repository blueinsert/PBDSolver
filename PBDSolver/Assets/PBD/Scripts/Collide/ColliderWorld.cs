
using bluebean.Physics.PBD.DataStruct;
using bluebean.Physics.PBD.DataStruct.Native;
using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
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

    /// <summary>
    /// 存放unity中的碰撞体数据；
    /// 这些碰撞体会与pbd粒子发生碰撞
    /// </summary>
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
        /// <summary>
        /// 前端对象引用
        /// </summary>
        [NonSerialized] public List<ColliderHandle> m_colliderHandles;
        [NonSerialized] public NativeColliderShapeList m_colliderShapes;         // list of collider shapes.
        [NonSerialized] public NativeAabbList m_colliderAabbs;                   // list of collider bounds.
        [NonSerialized] public NativeAffineTransformList m_colliderTransforms;   // list of collider transforms.
        /// <summary>
        /// 碰撞体的三角网格数据容器，如果shapeType为TriangleMesh
        /// </summary>
        [NonSerialized] public TriangleMeshContainer m_triangleMeshContainer;
        /// <summary>
        /// 每个碰撞体在网格坐标上跨域的范围
        /// </summary>
        [NonSerialized] public NativeCellSpanList m_colliderCellSpans;

        private int m_colliderCount = 0;
        #endregion

        #region 碰撞体网格划分，空间优化
        private NativeQueue<MovingCollider> m_movingColliders;
        private NativeMultilevelGrid<int> m_grid;
        #endregion

        /// <summary>
        /// 碰撞接触
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

            m_colliderShapes.Add(new ColliderShape() { });
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

        public void UpdateColliderData(int index, ColliderShape shape,Aabb aabb,AffineTransform transform)
        {
            m_colliderShapes[index] = shape;
            m_colliderAabbs[index] = aabb;
            m_colliderTransforms[index] = transform;
        }

        /// <summary>
        /// 更新碰撞体数据
        /// </summary>
        private void UpdateColliders()
        {
            // update all colliders:
            for (int i = 0; i < m_colliderHandles.Count; ++i)
                m_colliderHandles[i].owner.UpdateIfNeeded();
        }

        /// <summary>
        /// 使用多重网格，更新碰撞体的空间划分
        /// </summary>
        private void UpdateCollidersMultiGrid(float deltaTime)
        {
            if (m_colliderCount <= 0)
                return;
            //获取移动的碰撞体和其范围
            var identifyMoving = new IdentifyMovingCollidersJob
            {
                //输入
                shapes = this.m_colliderShapes.AsNativeArray<BurstColliderShape>(m_colliderCellSpans.count),
                //rigidbodies = world.rigidbodies.AsNativeArray<BurstRigidbody>(),
                //collisionMaterials = world.collisionMaterials.AsNativeArray<BurstCollisionMaterial>(),
                bounds = this.m_colliderAabbs.AsNativeArray<BurstAabb>(m_colliderCellSpans.count),
                //colliderCount = m_colliderCount,
                dt = deltaTime,
                //输出
                movingColliders = this.m_movingColliders.AsParallelWriter(),
                colliderCellSpans = this.m_colliderCellSpans.AsNativeArray<BurstCellSpan>(),
            };
            JobHandle movingHandle = identifyMoving.Schedule(m_colliderCount, 128);
            //更新multiGrid
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
        /// 碰撞侦测，产生接触
        /// </summary>
        /// <param name="deltaTime"></param>
        /// <param name="inputDeps"></param>
        /// <returns></returns>
        public JobHandle GenerateContacts(float deltaTime, JobHandle inputDeps)
        {

            var generateColliderContactsJob = new GenerateContactsJob
            {
                //输入
                colliderMultiGrid = m_grid,
                gridLevels = m_grid.populatedLevels.GetKeyArray(Allocator.TempJob),

                positions = Solver.ParticlePositions,
                velocities = Solver.ParticleVels,
                invMasses = Solver.InvMasses,
                radii = Solver.ParticleRadius,
                particleBounds = Solver.ParticleAabb,

                colliderTransforms = this.m_colliderTransforms.AsNativeArray<BurstAffineTransform>(),
                colliderShapes = this.m_colliderShapes.AsNativeArray<BurstColliderShape>(),
                //rigidbodies = world.rigidbodies.AsNativeArray<BurstRigidbody>(),
                colliderBounds = this.m_colliderAabbs.AsNativeArray<BurstAabb>(),

                triangleMeshHeaders = this.m_triangleMeshContainer.headers.AsNativeArray<TriangleMeshHeader>(),
                 triangleMesh_bihNodes = this.m_triangleMeshContainer.bihNodes.AsNativeArray<BIHNode>(),
                 triangleMesh_triangles = this.m_triangleMeshContainer.triangles.AsNativeArray<Triangle>(),
                 triangleMesh_vertices = this.m_triangleMeshContainer.vertices.AsNativeArray<float3>(),

                deltaTime = deltaTime,

                //输出
                contactsQueue = this.m_colliderContactQueue.AsParallelWriter(),
            };

            return generateColliderContactsJob.Schedule(Solver.ParticlePositions.Count(), 16, inputDeps);

        }
    }
}
