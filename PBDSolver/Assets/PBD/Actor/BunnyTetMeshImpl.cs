using bluebean.UGFramework.DataStruct;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.UGFramework.Physics
{
    public class BunnyTetMeshImpl : TetMesh
    {
        public void Awake()
        {
            Init();
        }

        public override string GetMeshName()
        {
            return "BunnyMesh";
        }

        public override void Init()
        {
            m_numParticles = BunnyMeshData.Verts.Length / 3;
            m_numEdges = BunnyMeshData.TetEdgeIds.Length / 2;
            m_numTets = BunnyMeshData.TetIds.Length / 4;
            m_numSurfs = BunnyMeshData.TetSurfaceTriIds.Length / 3;
            m_tetSurfaceTriIds = new int[BunnyMeshData.TetSurfaceTriIds.Length];
            for (int i = 0; i < BunnyMeshData.TetSurfaceTriIds.Length; i++)
            {
                m_tetSurfaceTriIds[i] = BunnyMeshData.TetSurfaceTriIds[i];
            }

            m_pos = new Vector3[m_numParticles];
            for (int i = 0; i < m_numParticles; i++)
            {
                float x = (float)BunnyMeshData.Verts[i * 3 + 0];
                float y = (float)BunnyMeshData.Verts[i * 3 + 1];
                float z = (float)BunnyMeshData.Verts[i * 3 + 2];
                m_pos[i] = new Vector3(x, y, z);
            }

            m_tet = new VectorInt4[m_numTets];
            for (int i = 0; i < m_numTets; i++)
            {
                int p1 = BunnyMeshData.TetIds[i * 4];
                int p2 = BunnyMeshData.TetIds[i * 4 + 1];
                int p3 = BunnyMeshData.TetIds[i * 4 + 2];
                int p4 = BunnyMeshData.TetIds[i * 4 + 3];
                m_tet[i] = new VectorInt4(p1, p2, p3, p4);
            }

            m_edge = new Vector2Int[m_numEdges];
            for (int i = 0; i < m_numEdges; i++)
            {
                int p1 = BunnyMeshData.TetEdgeIds[i * 2];
                int p2 = BunnyMeshData.TetEdgeIds[i * 2 + 1];
                m_edge[i] = new Vector2Int(p1, p2);
            }

            m_surf = new Vector3Int[m_numSurfs];
            for (int i = 0; i < m_numSurfs; i++)
            {
                int p1 = BunnyMeshData.TetSurfaceTriIds[i * 3];
                int p2 = BunnyMeshData.TetSurfaceTriIds[i * 3 + 1];
                int p3 = BunnyMeshData.TetSurfaceTriIds[i * 3 + 2];
                m_surf[i] = new Vector3Int(p1, p2, p3);
            }
            m_restVol = new float[m_numTets];
            m_restLen = new float[m_numEdges];
            m_invMass = new float[m_numParticles];
            m_mass = new float[m_numParticles];
            InitPhysics();
            m_isInitialized = true;
        }

        private void InitPhysics()
        {
            for (int i = 0; i < m_numTets; i++)
            {
                m_restVol[i] = TetVolume(i);
            }
            for (int i = 0; i < m_numEdges; i++)
            {
                var edge = m_edge[i];
                var id1 = edge.x;
                var id2 = edge.y;
                m_restLen[i] = (m_pos[id1] - m_pos[id2]).magnitude;
            }

            for (int i = 0; i < m_numTets; i++)
            {
                var v = m_restVol[i];
                var pMass = v / 4.0f;
                var id1 = m_tet[i].x;
                var id2 = m_tet[i].y;
                var id3 = m_tet[i].z;
                var id4 = m_tet[i].w;

                m_mass[id1] += pMass;
                m_mass[id2] += pMass;
                m_mass[id3] += pMass;
                m_mass[id4] += pMass;
            }
            for (int i = 0; i < m_numParticles; i++)
            {
                m_invMass[i] = 1.0f / (Mathf.Max(0.01f, m_mass[i]));
            }
        }
    }
}