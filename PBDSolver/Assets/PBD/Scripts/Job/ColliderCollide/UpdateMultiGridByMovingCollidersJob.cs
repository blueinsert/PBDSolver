using bluebean.Physics.PBD.DataStruct.Native;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

namespace bluebean.Physics.PBD
{
    /// <summary>
    /// 更新Collider的空间划分，使用MultiGrid;
    /// 根据碰撞体的size，将其id存入对应level的、大小覆盖范围内的格子中
    /// </summary>
    [BurstCompile]
    public struct UpdateMultiGridByMovingCollidersJob : IJob
    {
        public NativeQueue<MovingCollider> movingColliders;
        public NativeMultilevelGrid<int> grid;
        [ReadOnly] public int colliderCount;

        public void Execute()
        {
            while (movingColliders.Count > 0)
            {
                MovingCollider movingCollider = movingColliders.Dequeue();

                // remove from old cells:
                grid.RemoveFromCells(movingCollider.m_oldSpan, movingCollider.m_entity);

                // insert in new cells, as long as the index is below the amount of colliders.
                // otherwise, the collider is at the "tail" and there's no need to add it back.
                if (movingCollider.m_entity < colliderCount)
                    grid.AddToCells(movingCollider.m_newSpan, movingCollider.m_entity);
            }

            // remove all empty cells from the grid:
            grid.RemoveEmpty();
        }
    }
}
