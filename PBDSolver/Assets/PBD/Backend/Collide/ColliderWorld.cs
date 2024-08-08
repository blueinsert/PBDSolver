
using bluebean.Physics.PBD.DataStruct;
using bluebean.Physics.PBD.DataStruct.Native;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
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
        struct MovingCollider
        {
            public BurstCellSpan oldSpan;
            public BurstCellSpan newSpan;
            public int entity;
        }

        #region �ڲ�����

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

        public void Initialzie()
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
        public void UpdateCollidersMultiGrid()
        {

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
