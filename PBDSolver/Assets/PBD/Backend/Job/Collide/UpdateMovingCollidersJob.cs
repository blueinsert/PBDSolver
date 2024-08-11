using bluebean.Physics.PBD.DataStruct.Native;
using bluebean.Physics.PBD;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    /// <summary>
    /// ����MovingCollider����MultiGrid
    /// </summary>
    [BurstCompile]
    public struct UpdateMovingCollidersJob : IJob
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
                grid.RemoveFromCells(movingCollider.oldSpan, movingCollider.entity);

                // insert in new cells, as long as the index is below the amount of colliders.
                // otherwise, the collider is at the "tail" and there's no need to add it back.
                if (movingCollider.entity < colliderCount)
                    grid.AddToCells(movingCollider.newSpan, movingCollider.entity);
            }

            // remove all empty cells from the grid:
            grid.RemoveEmpty();
        }
    }
}