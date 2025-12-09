using bluebean.Physics.PBD.DataStruct;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct BatchContactsJob : IJob
    {
        [DeallocateOnJobCompletion]
        public NativeArray<ushort> batchMasks;

        [DeallocateOnJobCompletion]
        public NativeArray<int> batchIndices;

        [ReadOnly] public BatchLUT lut;
        public ContactProvider constraintDesc;
        public NativeArray<BatchData> batchData;
        public NativeArray<int> activeBatchCount;

        public int maxBatches;

        public unsafe void Execute()
        {
            // Initialize batch data array
            for (int i = 0; i < batchData.Length; ++i)
                batchData[i] = new BatchData(i, maxBatches);

            // temporary array containing an open work item for each batch.
            WorkItem* workItems = stackalloc WorkItem[maxBatches];
            for (int i = 0; i < maxBatches; i++)
                workItems[i] = new WorkItem();

            int constraintCount = constraintDesc.GetConstraintCount();

            // find a batch for each constraint:
            for (int i = 0; i < constraintCount; ++i)
            {
                // OR together the batch masks of all entities involved in the constraint:
                int batchMask = 0;
                for (int k = 0; k < constraintDesc.GetParticleCount(i); ++k)
                    batchMask |= batchMasks[constraintDesc.GetParticle(i, k)];

                // look up the first free batch index for this constraint:
                int batchIndex = batchIndices[i] = lut.batchIndex[batchMask];

                // update the amount of constraints in the batch:
                var batch = batchData[batchIndex];
                batch.constraintCount++;
                batchData[batchIndex] = batch;

                // add the constraint to the last work item of the batch:
                if (workItems[batchIndex].Add(i))
                {
                    // if this work item does not belong to the last batch:
                    if (batchIndex != maxBatches - 1)
                    {
                        // tag all entities in the work item with the batch mask to close it.
                        // this way we know constraints referencing any of these entities can no longer be added to this batch.
                        for (int j = 0; j < workItems[batchIndex].constraintCount; j++)
                        {
                            int constraint = workItems[batchIndex].constraints[j];

                            for (int k = 0; k < constraintDesc.GetParticleCount(constraint); ++k)
                                batchMasks[constraintDesc.GetParticle(constraint, k)] |= batch.batchID;
                        }
                    }

                    // reuse the work item.
                    workItems[batchIndex].constraintCount = 0;
                }

            }

            // fill batch data:
            activeBatchCount[0] = 0;
            int numConstraints = 0;
            for (int i = 0; i < batchData.Length; ++i)
            {
                var batch = batchData[i];

                // bail out when we find the first empty batch:
                if (batch.constraintCount == 0)
                    break;

                // calculate work item size, count, and index of the first constraint
                batch.workItemSize = math.min(WorkItem.minWorkItemSize, batch.constraintCount);
                batch.workItemCount = (batch.constraintCount + batch.workItemSize - 1) / batch.workItemSize;
                batch.startIndex = numConstraints;

                numConstraints += batch.constraintCount;
                activeBatchCount[0]++;

                batchData[i] = batch;
            }

            // write out sorted constraint indices:
            for (int i = 0; i < constraintCount; ++i)
            {
                var batch = batchData[batchIndices[i]];
                int sortedIndex = batch.startIndex + (batch.activeConstraintCount++);
                constraintDesc.WriteSortedConstraint(i, sortedIndex);
                batchData[batchIndices[i]] = batch;
            }

        }


    }
}