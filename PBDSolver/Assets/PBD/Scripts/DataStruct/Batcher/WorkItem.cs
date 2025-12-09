using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct
{
    public unsafe struct WorkItem
    {
        public const int minWorkItemSize = 64;
        public fixed int constraints[minWorkItemSize];
        public int constraintCount;

        public bool Add(int constraintIndex)
        {
            // add the constraint to this work item.
            fixed (int* constraintIndices = constraints)
            {
                constraintIndices[constraintCount] = constraintIndex;
            }

            // if we've completed the work item, close it and reuse for the next one.
            return (++constraintCount == minWorkItemSize);
        }
    }
}
