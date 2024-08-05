using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.UGFramework.Physics
{
    [RequireComponent(typeof(TetMesh))]
    public class SoftBodyActor : PDBActor
    {
        const int CollideConstrainCountMax = 500;
        //CollideConstrain[] m_collideConstrains = new CollideConstrain[CollideConstrainCountMax];
        int m_collideConstrainCount = 0;

        private float m_scale = 1.0f;

        protected TetMesh m_tetMesh = null;

        public MeshFilter m_meshFilter = null;
        public Mesh m_mesh = null;

        Vector3[] m_x;

        // Start is called before the first frame update
        void Start()
        {
            Initialize();
        }

        public override void Initialize()
        {
            base.Initialize();
            m_tetMesh = GetComponent<TetMesh>();

            m_meshFilter = GetComponent<MeshFilter>();
            m_mesh = m_meshFilter.mesh;

            m_x = new Vector3[GetParticleCount()];
            for (int i = 0; i < m_x.Length; i++)
            {
                m_x[i] = m_tetMesh.GetParticlePos(i);
            }
            m_solver.AddActor(this);
            PushStretchConstrains2Solver();
            PushVolumeConstrains2Solver();
        }

        public override void OnPreSubStep(float dt, Vector3 g)
        {
           
        }

        void GenerateCollideConstrains()
        {
            m_collideConstrainCount = 0;

            for (int i = 0; i < m_x.Length; i++)
            {
                var p = m_x[i];
                float planeY = -5;
                if (p.y < planeY && m_collideConstrainCount < CollideConstrainCountMax - 1)
                {
                    //m_collideConstrains[m_collideConstrainCount].m_actorId = this.ActorId;
                    //m_collideConstrains[m_collideConstrainCount].m_index = i;
                    //m_collideConstrains[m_collideConstrainCount].m_normal = new Vector3(0, 1, 0);
                    //m_collideConstrains[m_collideConstrainCount].m_entryPosition = new Vector3(p.x, planeY, p.z);
                    m_collideConstrainCount++;
                }
            }
        }

        void PushCollideConstrains2Solver()
        {
            for (int i = 0; i < m_collideConstrainCount; i++)
            {
                //var constrain = m_collideConstrains[i];
                //m_solver.AddConstrain(constrain);
            }
        }

        void PushStretchConstrains2Solver()
        {
            for (int e = 0; e < m_tetMesh.m_numEdges; e++)
            {
                var edge = m_tetMesh.m_edge[e];
                StretchConstrainData constrain = new StretchConstrainData()
                {
                    m_actorId = this.ActorId,
                    m_edge = new DataStruct.VectorInt2(edge.x, edge.y),
                    m_restLen = m_tetMesh.GetEdgeRestLen(e),
                };
                m_solver.PushStretchConstrain(constrain);
            }
        }

        void PushVolumeConstrains2Solver()
        {
            for (int i = 0; i < m_tetMesh.m_numTets; i++)
            {
                var tet = m_tetMesh.m_tet[i];
                VolumeConstrainData constrain = new VolumeConstrainData()
                {
                    m_actorId = this.ActorId,
                    m_tet = new DataStruct.VectorInt4(tet.x, tet.y, tet.z, tet.w),
                    m_restVolume = m_tetMesh.GetTetRestVolume(i),
                };
                m_solver.PushVolumeConstrain(constrain);
            }
        }

        public float GetTetRestVolume(int tetIndex)
        {
            var volume = m_tetMesh.GetTetRestVolume(tetIndex);
            var tet = m_tetMesh.m_tet[tetIndex];
            var fixedCount = 0;
            fixedCount += m_tetMesh.IsParticleFixed(tet[0]) ? 1 : 0;
            fixedCount += m_tetMesh.IsParticleFixed(tet[1]) ? 1 : 0;
            fixedCount += m_tetMesh.IsParticleFixed(tet[2]) ? 1 : 0;
            fixedCount += m_tetMesh.IsParticleFixed(tet[3]) ? 1 : 0;
            switch (fixedCount)
            {
                case 0:
                    volume = volume * m_scale * m_scale * m_scale;
                    break;
                case 1:
                    volume = volume * m_scale * m_scale * m_scale;
                    break;
                case 2:
                    volume = volume * m_scale * m_scale;
                    break;
                case 3:
                    volume = volume * m_scale;
                    break;
                case 4: break;
            }
            return volume;
        }

        public float GetEdgeRestLen(int edgeIndex)
        {
            var edge = m_tetMesh.m_edge[edgeIndex];
            var len = m_tetMesh.GetEdgeRestLen(edgeIndex);
            if (m_tetMesh.IsParticleFixed(edge[0]) && m_tetMesh.IsParticleFixed(edge[1]))
            {
                return len;
            }
            return len * m_scale;
        }

        public void Update()
        {

            float speed = 3.0f;
            if (Input.GetKey(KeyCode.O))
            {
                m_scale += speed * Time.deltaTime;
            }
            if (Input.GetKey(KeyCode.P))
            {
                m_scale -= speed * Time.deltaTime;
            }
            m_scale = Mathf.Clamp(m_scale, 1.0f, 20.0f);
        }

        public override void OnPostStep()
        {
            SyncMesh();
        }


        public void SyncMesh()
        {
            for(int i = 0; i < m_particleIndicesInSolver.Length; i++)
            {
                var globalIndex = m_particleIndicesInSolver[i];
                m_x[i] = m_solver.GetParticlePosition(globalIndex);
            }
            //Debug.Log($"m_x[0] {m_x[0]}");
            m_mesh.vertices = m_x;
            m_mesh.RecalculateNormals();
        }

        public override int GetParticleCount()
        {
            return m_tetMesh.m_pos.Length;
        }

        public override Vector3 GetParticleInitPosition(int particleIndex)
        {
            return m_tetMesh.GetParticlePos(particleIndex);
        }

        public override float GetParticleInvMass(int particleIndex)
        {
            return m_tetMesh.m_invMass[particleIndex];
        }
    }
}