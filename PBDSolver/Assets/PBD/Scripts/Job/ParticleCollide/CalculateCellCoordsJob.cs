using bluebean.Physics.PBD.DataStruct.Native;
using bluebean.Physics.PBD.DataStruct;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    struct CalculateCellCoordsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<BurstAabb> simplexBounds;
        public NativeArray<int4> cellCoords;
        [ReadOnly] public bool is2D;

        public void Execute(int i)
        {
            int level = NativeMultilevelGrid<int>.GridLevelForSize(simplexBounds[i].AverageAxisLength());
            float cellSize = NativeMultilevelGrid<int>.CellSizeOfLevel(level);

            // get new particle cell coordinate:
            int4 newCellCoord = new int4(GridHash.Quantize(simplexBounds[i].center.xyz, cellSize), level);

            // if the solver is 2D, project the particle to the z = 0 cell.
            if (is2D) newCellCoord[2] = 0;

            cellCoords[i] = newCellCoord;
        }
    }
}
