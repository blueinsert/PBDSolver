using System;
using System.Collections.Generic;
using UnityEngine;
using bluebean.Physics.PBD.DataStruct;

namespace bluebean.Physics.PBD
{

    public class MeshShapeTracker : ShapeTracker
    {
        TriangleMeshHandle handle;

        public MeshShapeTracker(PBDColliderBase source, MeshCollider collider)
        {

            this.source = source;
            this.collider = collider;
        }

        private ColliderWorld ColliderWorld { 
            get {
                var solver = this.source.GetComponentInParent<PBDSolver>();
                if (solver != null)
                {
                    return solver.ColliderWorld;
                }
                return null;
            } 
        }

        /**
		 * Forces the tracker to update mesh data during the next call to UpdateIfNeeded().
		 */
        public void UpdateMeshData()
        {
            ColliderWorld.DestroyTriangleMesh(handle);
        }

        public override bool UpdateIfNeeded()
        {

            MeshCollider meshCollider = collider as MeshCollider;

            // retrieve collision world and index:
            var world = ColliderWorld;
            int index = source.Handle.index;

            // decrease reference count of current handle if the mesh data it points to is different
            // than the mesh used by the collider:
            if (handle != null && handle.owner != meshCollider.sharedMesh)
            {
                if (handle.Dereference())
                    world.DestroyTriangleMesh(handle);
            }

            // get or create the mesh:
            if (handle == null || !handle.isValid)
            {
                handle = world.GetOrCreateTriangleMesh(meshCollider.sharedMesh);
                handle.Reference();
            }

            // update collider:
            var shape = world.m_colliderShapes[index];
            shape.type = ColliderShapeType.TriangleMesh;
            shape.rigidbodyIndex = source.Rigidbody != null ? source.Rigidbody.handle.index : -1;
            shape.contactOffset = 0.01f;
            shape.dataIndex = handle.index;
            world.m_colliderShapes[index] = shape;

            // update bounds:
            var aabb = world.m_colliderAabbs[index];
            aabb.FromBounds(meshCollider.bounds, shape.contactOffset);
            world.m_colliderAabbs[index] = aabb;

            // update transform:
            var trfm = world.m_colliderTransforms[index];
            trfm.FromTransform(meshCollider.transform);
            world.m_colliderTransforms[index] = trfm;

            return true;
        }

        public override void Destroy()
        {
            base.Destroy();

            if (handle != null && handle.Dereference())
                ColliderWorld.DestroyTriangleMesh(handle);
        }
    }
}

