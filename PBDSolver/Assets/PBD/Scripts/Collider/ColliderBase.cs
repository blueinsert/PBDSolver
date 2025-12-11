using UnityEngine;

namespace bluebean.Physics.PBD
{
    public class ColliderBase : MonoBehaviour
    {
        private ISolver m_solver = null;
        protected ColliderHandle m_colliderHandle = null;
        protected ISolver Solver
        {
            get
            {
                if (m_solver == null)
                {
                    m_solver = GetComponentInParent<ISolver>();
                }
                return m_solver;
            }
        }

        public void AddCollider()
        {
            if (m_colliderHandle == null)
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
        /// </summary>
        public virtual void UpdateIfNeeded()
        {

        }

    }
}
