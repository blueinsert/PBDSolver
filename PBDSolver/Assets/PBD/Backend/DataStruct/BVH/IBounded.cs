using System;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct
{
    public interface IBounded
    {
        Aabb GetBounds();
    }
}
