using bluebean.Physics.PBD.DataStruct;
using bluebean.Physics.PBD.DataStruct.Native;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    /// <summary>
    /// 标识移动的碰撞体，
    /// 移动的碰撞体才需要更新所在网格
    /// </summary>
    [BurstCompile]
    public struct IdentifyMovingCollidersJob : IJobParallelFor
    {
        [WriteOnly]
        [NativeDisableParallelForRestriction]
        public NativeQueue<MovingCollider>.ParallelWriter movingColliders;

        [ReadOnly] public NativeArray<BurstColliderShape> shapes;
        //[ReadOnly] public NativeArray<BurstRigidbody> rigidbodies;
        //[ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;
        /// <summary>
        /// 碰撞体Aabb世界坐标
        /// </summary>
        public NativeArray<BurstAabb> bounds;

        public NativeArray<BurstCellSpan> cellIndices;
        [ReadOnly] public int colliderCount;
        [ReadOnly] public float dt;

        public void Execute(int i)
        {
            BurstAabb velocityBounds = bounds[i];

            //int rb = shapes[i].rigidbodyIndex;

            // Expand bounds by rigidbody's linear velocity
            // (check against out of bounds rigidbody access, can happen when a destroyed collider references a rigidbody that has just been destroyed too)
            //if (rb >= 0 && rb < rigidbodies.Length)
            //    velocityBounds.Sweep(rigidbodies[rb].velocity * dt);

            // Expand bounds by collision material's stick distance:
            //if (shapes[i].materialIndex >= 0)
            //    velocityBounds.Expand(collisionMaterials[shapes[i].materialIndex].stickDistance);

            float size = velocityBounds.AverageAxisLength();
            int level = NativeMultilevelGrid<int>.GridLevelForSize(size);
            float cellSize = NativeMultilevelGrid<int>.CellSizeOfLevel(level);

            // get new collider bounds cell coordinates:
            BurstCellSpan newSpan = new BurstCellSpan(new int4(GridHash.Quantize(velocityBounds.min.xyz, cellSize), level),
                                                      new int4(GridHash.Quantize(velocityBounds.max.xyz, cellSize), level));

            // if the collider is 2D, project it to the z = 0 cells.
            //if (shapes[i].is2D != 0)
            //{
            //    newSpan.min[2] = 0;
            //    newSpan.max[2] = 0;
            //}

            // if the collider is at the tail (removed), we will only remove it from its current cellspan.
            // if the new cellspan and the current one are different, we must remove it from its current cellspan and add it to its new one.
            if (i >= colliderCount || cellIndices[i] != newSpan)
            {
                // Add the collider to the list of moving colliders:
                movingColliders.Enqueue(new MovingCollider()
                {
                    oldSpan = cellIndices[i],
                    newSpan = newSpan,
                    entity = i
                });

                // Update previous coords:
                cellIndices[i] = newSpan;
            }
        }
    }
}
