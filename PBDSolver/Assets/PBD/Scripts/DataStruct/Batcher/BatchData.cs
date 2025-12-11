using Unity.Mathematics;

namespace bluebean.Physics.PBD.DataStruct
{
    public struct BatchData
    {
        public ushort batchID;             // Batch identifier. All bits will be '0', except for the one at the position of the batch.

        public int startIndex;             // first constraint in the batch
        public int constraintCount;        // amount of constraints in the batch.
        public int activeConstraintCount;  // auxiliar counter used to sort the constraints in linear time.

        //设计目的是为了并行处理
        public int workItemSize;           // size of each work item.
        public int workItemCount;          // number of work items.
        public bool isLast;

        public BatchData(int index, int maxBatches)
        {
            batchID = (ushort)(1 << index);
            isLast = index == (maxBatches - 1);
            constraintCount = 0;
            activeConstraintCount = 0;

            startIndex = 0;
            workItemSize = 0;
            workItemCount = 0;
        }

        public void GetConstraintRange(int workItemIndex, out int start, out int end)
        {
            start = startIndex + workItemSize * workItemIndex;
            end = startIndex + math.min(constraintCount, workItemSize * (workItemIndex + 1));
        }

    }
}