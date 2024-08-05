using System;
using UnityEngine;

namespace bluebean.UGFramework.DataStruct
{
    public interface IBounded
    {
        Aabb GetBounds();
    }
}
