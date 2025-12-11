using bluebean.Physics.PBD.DataStruct.Native;
using bluebean.Physics.PBD.DataStruct;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using Unity.Jobs;
using System.Linq;

namespace bluebean.Physics.PBD
{
    /// <summary>
    /// 粒子之间碰撞相关
    /// </summary>
    public class ParticleGrid : IDisposable
    {
        public NativeMultilevelGrid<int> m_grid;
        public NativeQueue<BurstContact> m_particleContactQueue;

        public ParticleGrid()
        {
            this.m_grid = new NativeMultilevelGrid<int>(1000, Allocator.Persistent);
            this.m_particleContactQueue = new NativeQueue<BurstContact>(Allocator.Persistent);
        }

        public void Update(ISolver solver, float deltaTime, JobHandle inputDeps)
        {
            var particleCount = solver.ParticlePositions.Count();
            var calculateCells = new CalculateCellCoordsJob
            {
                particleBounds = solver.ParticleAabb,
                cellCoords = solver.CellCoords,
                is2D = false,
            };

            inputDeps = calculateCells.Schedule(particleCount, 4, inputDeps);
            
            var updateGrid = new UpdateParticleGridJob
            {
                grid = m_grid,
                cellCoords = solver.CellCoords,
                particleCount = particleCount,
            };
            updateGrid.Schedule(inputDeps).Complete();
        }

        public JobHandle GenerateContacts(ISolver solver, float deltaTime)
        {

            var generateParticleContactsJob = new GenerateParticleParticleContactsJob
            {
                grid = m_grid,
                gridLevels = m_grid.populatedLevels.GetKeyArray(Allocator.TempJob),

                positions = solver.ParticlePositions,
                //restPositions = solver.res,//todo
                velocities = solver.ParticleVels,
                invMasses = solver.InvMasses,
                radii = solver.ParticleRadius,
                //normals = solver.nor,
                groups = solver.Groups,
                simplexBounds = solver.ParticleAabb,


                contactsQueue = m_particleContactQueue.AsParallelWriter(),
                dt = deltaTime,

                //todo 确认数值
                collisionMargin = 0.1f,
                optimizationIterations = 13,
                optimizationTolerance = 1,
            };

            return generateParticleContactsJob.Schedule(m_grid.CellCount, 1);
        }


        public void Dispose()
        {
            m_grid.Dispose();
            m_particleContactQueue.Dispose();
        }
    }
}
