using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.UGFramework.Physics
{
    /*
    public class CollideConstrain : ConstrainBase
    {
        public float m_collideCompliance = 0.0f;

        public int m_actorId;
        public int m_index;//¶¥µãË÷Òý
        public Vector3 m_normal;
        public Vector3 m_entryPosition;

        public CollideConstrain() : base(ConstrainType.Collide)
        {
        }

        public void SetParams(float compliance)
        {
            m_collideCompliance = compliance;
        }

        public override void Solve(float dt)
        {
            var alpha = m_collideCompliance / (dt * dt);
            var pos = m_solveEnv.GetParticlePosition(m_actorId, m_index);
            float C = Vector3.Dot((pos - m_entryPosition), m_normal);
            if (C < 0)
            {
                Vector3 grads = m_normal;
                float invMass = m_solveEnv.GetParticleInvMass(m_actorId, m_index);
                float w = invMass;
                float s = -C / (w + alpha);
                Vector3 dp = invMass * s * grads;
                m_solveEnv.ModifyParticelPosition(m_actorId, m_index, dp);
            }
        }
    }
    */
}
