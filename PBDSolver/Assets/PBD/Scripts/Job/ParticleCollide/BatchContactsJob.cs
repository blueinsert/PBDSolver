using bluebean.Physics.PBD.DataStruct;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    /// <summary>
    /// 将约束分批，每个批次里的约束相互之间不冲突，即不包含相同的粒子。
    /// </summary>
    [BurstCompile]
    public struct BatchContactsJob : IJob
    {
        //临时变量
        /// <summary>
        /// 每个粒子对应的batcher掩码，数组长度等于粒子数量；
        /// 详细的说是使用到该粒子的所有约束的batcher掩码，因为粒子可能在多个batcher中被涉及
        /// </summary>
        [DeallocateOnJobCompletion]
        public NativeArray<ushort> batchMasks;
        /// <summary>
        /// 每个约束对应的batcher掩码，数组长度等于约束数量
        /// </summary>
        [DeallocateOnJobCompletion]
        public NativeArray<int> batchIndices;
        //输入
        [ReadOnly] public BatchLUT lut;
        public ContactProvider constraintDesc;
        public int maxBatches;
        //输出
        public NativeArray<BatchData> batchData;
        public NativeArray<int> activeBatchCount;


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
                //粒子可能被包含在其他的约束中,
                for (int k = 0; k < constraintDesc.GetParticleCount(i); ++k)
                    batchMask |= batchMasks[constraintDesc.GetParticle(i, k)];//这种情况下不为0
                //这个时刻batchMask记录了已经被使用了的batch情况
                //通过查找表返回第一个未被使用的batcher的index
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
                    //因为是从低位开始分配未使用的batcher，到了最后一个就没必要记录了
                    if (batchIndex != maxBatches - 1)
                    {
                        // tag all entities in the work item with the batch mask to close it.
                        // this way we know constraints referencing any of these entities can no longer be added to this batch.
                        //遍历该batcher中所有约束涉及的所有粒子，记录其batcher掩码
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