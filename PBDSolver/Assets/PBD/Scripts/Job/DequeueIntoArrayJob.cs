using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;


namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct DequeueIntoArrayJob<T> : IJob where T : unmanaged
    {
        public int StartIndex;
        public NativeQueue<T> InputQueue;
        [WriteOnly] public NativeArray<T> OutputArray;

        public void Execute()
        {
            int queueCount = InputQueue.Count;

            for (int i = StartIndex; i < StartIndex + queueCount; i++)
            {
                OutputArray[i] = InputQueue.Dequeue();
            }
        }
    }
}
