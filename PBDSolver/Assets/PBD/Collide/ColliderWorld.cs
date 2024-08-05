
using bluebean.UGFramework.DataStruct;
using bluebean.UGFramework.DataStruct.Native;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.UGFramework.Physics
{
    public class ColliderWorld
    {
        struct MovingCollider
        {
            public BurstCellSpan oldSpan;
            public BurstCellSpan newSpan;
            public int entity;
        }

        [NonSerialized] public NativeColliderShapeList colliderShapes;         // list of collider shapes.
        [NonSerialized] public NativeAabbList colliderAabbs;                   // list of collider bounds.
        [NonSerialized] public NativeAffineTransformList colliderTransforms;   // list of collider transforms.
        [NonSerialized] public TriangleMeshContainer triangleMeshContainer;

        private int colliderCount = 0;
        private NativeQueue<MovingCollider> movingColliders;
        private NativeMultilevelGrid<int> grid;
        public NativeCellSpanList cellSpans;

        public NativeArray<int> simplices;
        public NativeArray<BurstAabb> simplexBounds;
        public NativeArray<float4> principalRadii;

        public NativeArray<BurstContact> colliderContacts;
    }
}
