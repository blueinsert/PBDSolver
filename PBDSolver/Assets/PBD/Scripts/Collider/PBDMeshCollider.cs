using bluebean.Physics.PBD.DataStruct;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    [RequireComponent(typeof(MeshCollider))]
    public class PBDMeshCollider : ColliderBase
    {
        TriangleMeshHandle m_shapeHandle;
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
            if (m_shapeHandle == null)
            {
                m_shapeHandle = colliderWorld.GetOrCreateTriangleMesh(m_unityMeshCollider.sharedMesh);
                m_shapeHandle.Reference();
            }
            var index = m_colliderHandle.index;
            var shape = colliderWorld.m_colliderShapes[index];
            shape.type = ColliderShapeType.TriangleMesh;
            shape.dataIndex = m_shapeHandle.index;
            colliderWorld.m_colliderShapes[index] = shape;

            var aabb = colliderWorld.m_colliderAabbs[index];
            aabb.FromBounds(m_unityMeshCollider.bounds, 0);
            colliderWorld.m_colliderAabbs[index] = aabb;

            var trfm = colliderWorld.m_colliderTransforms[index];
            trfm.FromTransform(m_unityMeshCollider.transform);
            colliderWorld.m_colliderTransforms[index] = trfm;
        }
    }
}
