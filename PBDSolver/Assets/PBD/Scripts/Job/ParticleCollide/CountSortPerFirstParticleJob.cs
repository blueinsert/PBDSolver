using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Jobs;
using bluebean.Physics.PBD.DataStruct;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct CountSortPerFirstParticleJob : IJob
    {
        [ReadOnly][NativeDisableContainerSafetyRestriction] public NativeArray<BurstContact> input;
        public NativeArray<BurstContact> output;

        [NativeDisableContainerSafetyRestriction] public NativeArray<int> digitCount;

        public int maxDigits;
        public int maxIndex;

        public void Execute()
        {
            // no real need for a mask, just in case bad particle indices were passed that have more digits than maxDigits.
            int mask = (1 << maxDigits) - 1;

            // Count digits
            for (int i = 0; i < input.Length; i++)
            {
                digitCount[input[i].GetParticle(0) & mask]++;
            }

            // Calculate start index for each digit
            int prev = digitCount[0];
            digitCount[0] = 0;
            for (int i = 1; i <= maxIndex; i++)
            {
                int current = digitCount[i];
                digitCount[i] = digitCount[i - 1] + prev;
                prev = current;
            }

            // Copy elements into buckets based on particle index
            for (int i = 0; i < input.Length; i++)
            {
                int index = digitCount[input[i].GetParticle(0) & mask]++;
                if (index == 1 && input.Length == 1)
                {
                    output[0] = input[0];
                }
                output[index] = input[i];
            }
        }
    }
}