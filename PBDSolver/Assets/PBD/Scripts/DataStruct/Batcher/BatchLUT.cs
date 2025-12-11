using System;
using Unity.Collections;

namespace bluebean.Physics.PBD.DataStruct
{
    /// <summary>
    /// 输入一个位掩码UInut16，位掩码中每一位代表该位置的batcher的使用情况；
    /// 输出位掩码中第一个0位的位置，即第一个可用的batcher的索引；
    /// </summary>
    public struct BatchLUT : IDisposable
    {
        public readonly int numBatches;
        //空间换时间
        public readonly NativeArray<ushort> batchIndex;

        public BatchLUT(int numBatches)
        {
            this.numBatches = numBatches;

            batchIndex = new NativeArray<ushort>(UInt16.MaxValue + 1, Allocator.Persistent, NativeArrayOptions.ClearMemory);
            const ushort end = UInt16.MaxValue;
            ushort numBits = (ushort)(numBatches - 1);

            // For each entry in the table, compute the position of the first '0' bit in the index, starting from the less significant bit.
            // This is the index of the first batch where we can add the constraint to.

            for (ushort value = 0; value < end; value++)
            {
                ushort valueCopy = value;
                for (ushort i = 0; i < numBits; i++)
                {
                    if ((valueCopy & 1) == 0)
                    {
                        batchIndex[value] = i;
                        break;
                    }
                    valueCopy >>= 1;
                }

            }

            batchIndex[end] = numBits;
        }

        public void Dispose()
        {
            batchIndex.Dispose();
        }
    }
}