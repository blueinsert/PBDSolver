using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public class ColliderBase : MonoBehaviour
    {
        private PBDSolver m_solver = null;
        protected ColliderHandle m_colliderHandle = null;
        public PBDSolver Solver { 
            get
            {
                if(m_solver == null)
                {
                    m_solver = GetComponentInParent<PBDSolver>();
                }
                return m_solver;
            } 
        }

        public void AddCollider()
        {
            if(m_colliderHandle == null)
            {
                if (Solver != null)
                {
                    m_colliderHandle = Solver.ColliderWorld.CreateCollider();
                    m_colliderHandle.owner = this;
                }
            }
        }

        /// <summary>
        /// 向CollideWorld同步数据
        /// 从前端向后端
        /// </summary>
        public virtual void UpdateIfNeeded()
        {
            
        }

    }
}
