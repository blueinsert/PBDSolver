using System;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public abstract class ShapeTracker
    {
        protected PBDColliderBase source;
        protected Component collider;

        public virtual void Destroy()
        {
        }

        public abstract bool UpdateIfNeeded();

    }

}


