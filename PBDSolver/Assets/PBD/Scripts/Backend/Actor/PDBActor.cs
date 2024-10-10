using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    
    public abstract class PDBActor : MonoBehaviour
    {
        public int m_actorId;
        public int ActorId { get { return m_actorId; } }

        /// <summary>
        /// 记录了此actor的每个粒子在求解器数据数组中的索引
        /// </summary>
        //[HideInInspector]
        public int[] m_particleIndicesInSolver;

        protected ISolver m_solver = null;

        public abstract int GetParticleCount();

        public abstract Vector3 GetParticleInitPosition(int particleIndex);

        public abstract float GetParticleInvMass(int particleIndex);

        public virtual void Initialize()
        {
            m_solver = GetComponentInParent<ISolver>();
        }

        public virtual void OnPreSubStep(float dt, Vector3 g)
        {
            
        }

        public virtual void OnPostSubStep(float dt, float velDamp)
        {

        }

        public virtual void OnPreStep() { }

        public virtual void OnPostStep()
        {
        }

    }
}
