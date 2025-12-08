using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Jobs;
using bluebean.Physics.PBD.DataStruct;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct SortSubArraysJob : IJobParallelFor
    {
        [NativeDisableContainerSafetyRestriction] public NativeArray<BurstContact> InOutArray;

        // Typically lastDigitIndex is resulting RadixSortPerBodyAJob.digitCount. nextElementIndex[i] = index of first element with bodyA index == i + 1
        [NativeDisableContainerSafetyRestriction][DeallocateOnJobCompletion] public NativeArray<int> NextElementIndex;

        [ReadOnly] public BurstContactComparer comparer;

        public void Execute(int workItemIndex)
        {
            int startIndex = 0;
            if (workItemIndex > 0)
            {
                startIndex = NextElementIndex[workItemIndex - 1];
            }

            if (startIndex < InOutArray.Length)
            {
                int length = NextElementIndex[workItemIndex] - startIndex;
                DefaultSortOfSubArrays(InOutArray, startIndex, length, comparer);
            }
        }

        public static void DefaultSortOfSubArrays(NativeArray<BurstContact> inOutArray, int startIndex, int length, BurstContactComparer comparer)
        {
            if (length > 2)
            {
                var slice = inOutArray.Slice(startIndex, length);
                slice.Sort(comparer);
            }
            else if (length == 2) // just swap:
            {
                if (inOutArray[startIndex].GetParticle(1) > inOutArray[startIndex + 1].GetParticle(1))
                {
                    var temp = inOutArray[startIndex + 1];
                    inOutArray[startIndex + 1] = inOutArray[startIndex];
                    inOutArray[startIndex] = temp;
                }
            }
        }
    }
}