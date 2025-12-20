using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public enum DataSourceType
    {
        Mesh,
        TetMesh,
    }

    public class ShapeMatchingBasedSoftBodyActor : PBDActor
    {
        public DataSourceType m_dataSourceType = DataSourceType.Mesh;

        protected TetMesh m_tetMesh = null;
        MeshFilter m_meshFilter;
        Mesh m_mesh;

        Vector3[] m_x;

        Vector3[] rest_X;//√ø∂•µ„Œª÷√


        private void Start()
        {
            m_meshFilter = GetComponentInChildren<MeshFilter>();
            Mesh mesh = m_meshFilter.mesh;
            m_mesh = m_meshFilter.mesh;
            if(m_dataSourceType == DataSourceType.Mesh)
            {
                var len = mesh.vertices.Length;
                var l2w = m_meshFilter.transform.localToWorldMatrix;
                rest_X = new Vector3[len];
                m_x = new Vector3[len];
                for (int i = 0; i < len; i++)
                {
                    var p = l2w * mesh.vertices[i];
                    rest_X[i] = p;
                    m_x[i] = p;
                }
            }else
            {
                m_tetMesh = GetComponent<TetMesh>();
                var len = m_tetMesh.m_pos.Length;
                m_x = new Vector3[len];
                rest_X= new Vector3[len];
                for (int i = 0; i < m_x.Length; i++)
                {
                    m_x[i] = m_tetMesh.GetParticlePos(i);
                    rest_X[i] = m_tetMesh.GetParticlePos(i);
                }
            }
            
            Initialize();
        }

        public override void Initialize()
        {
            base.Initialize();
            m_solver.AddActor(this);

            m_solver.PushShapeMatchingConstrain(new ShapeMatchingConstrainData()
            {
                m_actorId = this.ActorId
            });
        }

        public override int GetParticleCount()
        {
            return rest_X.Length;
        }

        public override Vector3 GetParticleInitPosition(int particleIndex)
        {
            return rest_X[particleIndex];
        }

        public override float GetParticleInvMass(int particleIndex)
        {
            return 1;
        }

        public override float GetParticleRadius(int particleIndex)
        {
            return 0.1f;
        }

        public override void OnPostStep()
        {
            SyncMesh();
        }


        public void SyncMesh()
        {
            for (int i = 0; i < m_particleIndicesInSolver.Length; i++)
            {
                var globalIndex = m_particleIndicesInSolver[i];
                m_x[i] = m_solver.GetParticlePosition(globalIndex);
            }
            m_mesh.vertices = m_x;
            m_mesh.RecalculateNormals();
        }
    }
}
