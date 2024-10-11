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
        //输入
        [ReadOnly] public NativeArray<BurstColliderShape> shapes;
        //[ReadOnly] public NativeArray<BurstRigidbody> rigidbodies;
        //[ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;
        /// <summary>
        /// 碰撞体Aabb,世界坐标
        /// </summary>
        [ReadOnly] public NativeArray<BurstAabb> bounds;
        //[ReadOnly] public int colliderCount;
        [ReadOnly] public float dt;

        //输出
        /// <summary>
        /// 移动了的碰撞体数据
        /// </summary>
        [WriteOnly]
        [NativeDisableParallelForRestriction]
        public NativeQueue<MovingCollider>.ParallelWriter movingColliders;
        /// <summary>
        /// 碰撞体新的cellSpan
        /// </summary>
        public NativeArray<BurstCellSpan> colliderCellSpans;

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

            //计算collider的aabb在对应size的grid的坐标范围
            // get new collider bounds cell coordinates:
            BurstCellSpan newSpan = new BurstCellSpan(new int4(GridHash.Quantize(velocityBounds.min.xyz, cellSize), level),
                                                      new int4(GridHash.Quantize(velocityBounds.max.xyz, cellSize), level));

            // if the collider is at the tail (removed), we will only remove it from its current cellspan.
            // if the new cellspan and the current one are different, we must remove it from its current cellspan and add it to its new one.
            if (//i >= colliderCount || 
                colliderCellSpans[i] != newSpan)
            {
                // Add the collider to the list of moving colliders:
                movingColliders.Enqueue(new MovingCollider()
                {
                    m_oldSpan = colliderCellSpans[i],
                    m_newSpan = newSpan,
                    m_entity = i
                });

                // Update previous coords:
                colliderCellSpans[i] = newSpan;
            }
        }
    }
}
