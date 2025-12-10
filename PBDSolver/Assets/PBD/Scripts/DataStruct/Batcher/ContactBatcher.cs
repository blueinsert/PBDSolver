using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct
{
    public struct ContactBatcher : IDisposable
    {
        public int maxBatches;
        private BatchLUT batchLUT; // look up table for batch indices.

        public ContactBatcher(int maxBatches)
        {
            this.maxBatches = math.min(17, maxBatches);
            batchLUT = new BatchLUT(this.maxBatches);
        }

        public void Dispose()
        {
            batchLUT.Dispose();
        }

        /**
         * Linear-time graph coloring using bitmasks and a look-up table. Used to organize contacts into batches for parallel processing.
         * input: array of unsorted constraints.
         * output:
         * - sorted constraint indices array.
         * - array of batchData, one per batch: startIndex, batchSize, workItemSize (at most == batchSize), numWorkItems
         * - number of active batches.
         */

        public JobHandle BatchConstraints(ref ContactProvider constraintDesc,
                                             int particleCount,
                                             ref NativeArray<BatchData> batchData,
                                             ref NativeArray<int> activeBatchCount,
                                             JobHandle inputDeps)
        {
            if (activeBatchCount.Length != 1)
                return inputDeps;

            var batchJob = new BatchContactsJob
            {
                batchMasks = new NativeArray<ushort>(particleCount, Allocator.TempJob, NativeArrayOptions.ClearMemory),
                batchIndices = new NativeArray<int>(constraintDesc.GetConstraintCount(), Allocator.TempJob, NativeArrayOptions.ClearMemory),
                lut = batchLUT,
                constraintDesc = constraintDesc,
                batchData = batchData,
                activeBatchCount = activeBatchCount,
                maxBatches = maxBatches
            };

            return batchJob.Schedule(inputDeps);
        }


    }
}

