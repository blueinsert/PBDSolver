using bluebean.Physics.PBD.DataStruct.Native;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    struct UpdateGrid : IJob
    {
        public NativeMultilevelGrid<int> grid;
        [ReadOnly] public NativeArray<int4> cellCoords;
        [ReadOnly] public int particleCount;

        public void Execute()
        {
            grid.Clear();

            for (int i = 0; i < particleCount; ++i)
            {
                // add to new cell:
                int cellIndex = grid.GetOrCreateCell(cellCoords[i]);
                var newCell = grid.usedCells[cellIndex];
                newCell.Add(i);
                grid.usedCells[cellIndex] = newCell;
            }
        }
    }
}
