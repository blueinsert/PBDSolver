using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public enum ConstrainType
    {
        None = -2,
        Start,

        Collide,
        Volume,
        Stretch,

        Max,
    }
}
