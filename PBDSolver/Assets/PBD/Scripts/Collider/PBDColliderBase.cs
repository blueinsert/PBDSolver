using System;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public class PBDColliderBase : MonoBehaviour
    {
        private ISolver m_solver = null;
        protected ColliderHandle m_colliderHandle = null;
        protected ShapeTracker tracker;                               /**< tracker object used to determine when to update the collider's shape*/

        public PBDRigidbody Rigidbody
        {
            get { return pbdRigidbody; }
        }

        protected PBDRigidbody pbdRigidbody;

        public ColliderHandle Handle
        {
            get
            {
                return m_colliderHandle;
            }
        }

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

        protected virtual void CreateTracker()
        {
        }

        protected void CreateRigidbody()
        {

            pbdRigidbody = null;

            // find the first rigidbody up our hierarchy:
            Rigidbody rb = GetComponentInParent<Rigidbody>();

            // if we have an rigidbody above us, see if it has a ObiRigidbody component and add one if it doesn't:
            if (rb != null)
            {

                pbdRigidbody = rb.GetComponent<PBDRigidbody>();

                if (pbdRigidbody == null)
                    pbdRigidbody = rb.gameObject.AddComponent<PBDRigidbody>();

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

                    // Create shape tracker:
                    CreateTracker();

                    // Create rigidbody if necessary, and link ourselves to it:
                    CreateRigidbody();
                }
            }
        }

        //todo
        protected void RemoveCollider()
        {
            //ObiColliderWorld.GetInstance().DestroyCollider(shapeHandle);

            // Destroy shape tracker:
            if (tracker != null)
            {
                tracker.Destroy();
                tracker = null;
            }
        }

        /// <summary>
        /// 向CollideWorld同步数据
        /// </summary>
        public virtual void UpdateIfNeeded()
        {
            if (tracker != null)
                tracker.UpdateIfNeeded();
        }

    }
}
