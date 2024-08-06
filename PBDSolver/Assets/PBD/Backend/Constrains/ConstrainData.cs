using bluebean.Physics.PBD.DataStruct;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public struct StretchConstrainData
    {

        public int m_actorId;
        public VectorInt2 m_edge;
        public float m_restLen;

    }

    public struct VolumeConstrainData
    {
        public int m_actorId;
        public VectorInt4 m_tet;
        public float m_restVolume;
    }
}