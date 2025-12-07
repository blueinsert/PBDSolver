using bluebean.Physics.PBD.DataStruct;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    [RequireComponent(typeof(MeshCollider))]
    public class PBDMeshCollider : ColliderBase
    {
        TriangleMeshHandle m_triMeshHandle;
        private UnityEngine.MeshCollider m_unityMeshCollider = null;

        private void Awake()
        {
            m_unityMeshCollider = GetComponent<UnityEngine.MeshCollider>();
        }

        private void Start()
        {
            AddCollider();
        }

        public override void UpdateIfNeeded()
        {
            var colliderWorld = Solver.ColliderWorld;
            if (m_triMeshHandle == null)
            {
                m_triMeshHandle = colliderWorld.GetOrCreateTriangleMesh(m_unityMeshCollider.sharedMesh);
                m_triMeshHandle.Reference();
            }
            var index = m_colliderHandle.index;

            var shape = new ColliderShape();
            shape.type = ColliderShapeType.TriangleMesh;
            shape.dataIndex = m_triMeshHandle.index;
            var aabb = new Aabb();
            aabb.FromBounds(m_unityMeshCollider.bounds, 0);
            var trfm = new AffineTransform();
            trfm.FromTransform(m_unityMeshCollider.transform);

            colliderWorld.UpdateColliderData(index, shape, aabb, trfm);
        }
    }
}
