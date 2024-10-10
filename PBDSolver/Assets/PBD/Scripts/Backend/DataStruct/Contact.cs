using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Runtime.InteropServices;
using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct
{
    [StructLayout(LayoutKind.Sequential, Size = 144)]
    public struct Contact
    {
        public Vector4 pointA;
        public Vector4 pointB;         /**< Speculative point of contact. */
        public Vector4 normal;         /**< Normal direction. */
        public Vector4 tangent;        /**< Tangent direction. */
        public Vector4 bitangent;      /**< Bitangent direction. */

        public float distance;    /** distance between both colliding entities at the beginning of the timestep.*/

        public float normalImpulse;
        public float tangentImpulse;
        public float bitangentImpulse;
        public float stickImpulse;
        public float rollingFrictionImpulse;

        public int bodyA;    /** simplex index*/
        public int bodyB;    /** simplex or rigidbody index*/
    }
}
