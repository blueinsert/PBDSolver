using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public class SingleParticleActor : PBDActor
    {
        public float m_radius = 0.2f;
        public Vector3 m_x = Vector3.zero;

        private void Start()
        {
            this.transform.localScale = new Vector3(m_radius * 2, m_radius * 2, m_radius * 2);
            Initialize();
        }

        public override void Initialize()
        {
            base.Initialize();
            m_solver.AddActor(this);
        }

        public override void OnPostStep()
        {
            SyncView();
        }


        public void SyncView()
        {
            var globalIndex = m_particleIndicesInSolver[0];
            var x = m_solver.GetParticlePosition(globalIndex);
            m_x = x;
            this.transform.position = m_x;
        }

        public override int GetParticleCount()
        {
            return 1;
        }

        public override Vector3 GetParticleInitPosition(int particleIndex)
        {
            return this.transform.position;
        }

        public override float GetParticleInvMass(int particleIndex)
        {
            return 1;
        }

        public override float GetParticleRadius(int particleIndex)
        {
            return m_radius;
        }
    }
}