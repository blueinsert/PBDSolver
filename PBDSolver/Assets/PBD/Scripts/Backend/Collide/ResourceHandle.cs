using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace bluebean.Physics.PBD
{
    public class ResourceHandle<T> where T : class
    {
        public T owner = null;               /**< reference to the owner instance*/
        public int index = -1;               /**< index of this resource in the collision world.*/
        private int referenceCount = 0;      /**< amount of references to this handle. Can be used to clean up any associated resources after it reaches zero.*/

        public bool isValid
        {
            get { return index >= 0; }
        }

        public void Invalidate()
        {
            index = -1;
            referenceCount = 0;
        }

        public void Reference()
        {
            referenceCount++;
        }

        public bool Dereference()
        {
            return --referenceCount == 0;
        }

        public ResourceHandle(int index = -1)
        {
            this.index = index;
            owner = null;
        }
    }

    public class TriangleMeshHandle : ResourceHandle<Mesh>
    {
        public TriangleMeshHandle(Mesh mesh, int index = -1) : base(index) { owner = mesh; }
    }

    public class ColliderHandle : ResourceHandle<ColliderBase>
    {
        public ColliderHandle(int index = -1) : base(index) { }
    }
}
