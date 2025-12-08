using bluebean.Physics.PBD.DataStruct;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Animations;

namespace bluebean.Physics.PBD
{
    public struct BurstContactComparer : IComparer<BurstContact>
    {
        // Compares by Height, Length, and Width.
        public int Compare(BurstContact x, BurstContact y)
        {
            return x.GetParticle(1).CompareTo(y.GetParticle(1));
        }
    }
}
