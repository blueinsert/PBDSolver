using bluebean.Physics.PBD.DataStruct.Native;
using bluebean.Physics.PBD.DataStruct;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using System.Drawing;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct GenerateParticleParticleContactsJob : IJobParallelFor
    {
        [ReadOnly] public NativeMultilevelGrid<int> grid;
        [DeallocateOnJobCompletion]
        [ReadOnly] public NativeArray<int> gridLevels;

        [ReadOnly] public NativeArray<float4> positions;
        [ReadOnly] public NativeArray<float4> velocities;
        [ReadOnly] public NativeArray<float> invMasses;
        [ReadOnly] public NativeArray<float> radii;
        [ReadOnly] public NativeArray<int> groups;

        [ReadOnly] public NativeArray<BurstAabb> particleBounds;

        [ReadOnly] public float dt;
        [ReadOnly] public float collisionMargin;
        [ReadOnly] public int optimizationIterations;
        [ReadOnly] public float optimizationTolerance;

        [WriteOnly]
        [NativeDisableParallelForRestriction]
        public NativeQueue<BurstContact>.ParallelWriter contactsQueue;

        public void Execute(int i)
        {
            // Looks for close particles in the same cell:
            IntraCellSearch(i);

            // Looks for close particles in neighboring cells, in the same level or higher levels.
            IntraLevelSearch(i);
        }

        /// <summary>
        /// 单元格内粒子之间判断是否相交
        /// </summary>
        /// <param name="cellIndex"></param>
        private void IntraCellSearch(int cellIndex)
        {
            int cellLength = grid.usedCells[cellIndex].Length;

            for (int p = 0; p < cellLength; ++p)
            {
                for (int n = p + 1; n < cellLength; ++n)
                {
                    InteractionTest(grid.usedCells[cellIndex][p], grid.usedCells[cellIndex][n]);
                }
            }
        }

        /// <summary>
        /// 相邻的同一level的格子的粒子间判断相交
        /// </summary>
        /// <param name="cellIndex"></param>
        /// <param name="neighborCellIndex"></param>
        private void InterCellSearch(int cellIndex, int neighborCellIndex)
        {
            int cellLength = grid.usedCells[cellIndex].Length;
            int neighborCellLength = grid.usedCells[neighborCellIndex].Length;

            for (int p = 0; p < cellLength; ++p)
            {
                for (int n = 0; n < neighborCellLength; ++n)
                {
                    InteractionTest(grid.usedCells[cellIndex][p], grid.usedCells[neighborCellIndex][n]);
                }
            }
        }

        /// <summary>
        /// 在大于等于当前level的网格上，判断与邻居网格中的粒子是否发送相交
        /// </summary>
        /// <param name="cellIndex"></param>
        private void IntraLevelSearch(int cellIndex)
        {
            int4 cellCoords = grid.usedCells[cellIndex].Coords;

            //同一层级，在邻居网格中搜索；三维情况下共有26(3x3x3-1)各邻居；
            //搜索一半的邻居，另一半处理邻居时处理
            // neighboring cells in the current level:
            for (int i = 0; i < 13; ++i)
            {
                int4 neighborCellCoords = new int4(cellCoords.xyz + GridHash.cellOffsets3D[i], cellCoords.w);

                int neighborCellIndex;
                if (grid.TryGetCellIndex(neighborCellCoords, out neighborCellIndex))
                {
                    InterCellSearch(cellIndex, neighborCellIndex);
                }
            }

            // neighboring cells in levels above the current one:
            int levelIndex = gridLevels.IndexOf<int, int>(cellCoords.w);
            if (levelIndex >= 0)
            {
                //判断与更多size level的物体是否相交
                levelIndex++;
                for (; levelIndex < gridLevels.Length; ++levelIndex)
                {
                    int level = gridLevels[levelIndex];

                    // calculate index of parent cell in parent level:
                    //计算在当前level网格中的网格坐标
                    int4 parentCellCoords = NativeMultilevelGrid<int>.GetParentCellCoords(cellCoords, level);

                    // search in all neighbouring cells:
                    // 遍历可能与小单元格中粒子发生碰撞的大单元格中的27个；
                    for (int x = -1; x <= 1; ++x)
                        for (int y = -1; y <= 1; ++y)
                            for (int z = -1; z <= 1; ++z)
                            {
                                int4 neighborCellCoords = parentCellCoords + new int4(x, y, z, 0);

                                int neighborCellIndex;
                                if (grid.TryGetCellIndex(neighborCellCoords, out neighborCellIndex))
                                {
                                    InterCellSearch(cellIndex, neighborCellIndex);
                                }
                            }
                }
            }
        }

        private void InteractionTest(int A, int B)
        {
            if (A == B)
                return;
            int groupA = groups[A];
            int groupB = groups[B];
            // if all particles are in the same group:
            if (groupA == groupB)
            {
                return;
            }
            // skip the pair if their bounds don't intersect:
            if (!particleBounds[A].IntersectsAabb(particleBounds[B]))
                return;
            float rA = radii[A];
            float rB = radii[B];
            float4 pA = positions[A];
            float4 pB = positions[B];
            float4 velocityA = float4.zero, velocityB = float4.zero;
            velocityA = velocities[A];
            velocityB = velocities[B];
            float4 normal = math.normalizesafe(pA - pB);

            float dAB = math.dot(pA - pB, normal);
            float vel = math.dot(velocityA - velocityB, normal);

            // check if the projected velocity along the contact normal will get us within collision distance.
            if (vel * dt + dAB <= rA + rB + collisionMargin)
            {
                contactsQueue.Enqueue(new BurstContact()
                {
                    bodyA = A,
                    bodyB = B,
                    pointA = new float4(1, 0, 0, 0),
                    pointB = new float4(1, 0, 0, 0),
                    normal = normal,
                }); ;
            }

        }
    }
}