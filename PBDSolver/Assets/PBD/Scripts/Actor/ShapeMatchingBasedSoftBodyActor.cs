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
        [Range(0f, 1f)]
        public float m_staticFriction = 0.0f;
        [Range(0f, 1f)]
        public float m_dynamicFriction = 0.0f;
        public DataSourceType m_dataSourceType = DataSourceType.Mesh;

        protected TetMesh m_tetMesh = null;
        MeshFilter m_meshFilter;
        Mesh m_mesh;
        private Matrix4x4 m_initL2W = Matrix4x4.identity;

        Vector3[] m_x;

        Vector3[] rest_X;//√ø∂•µ„Œª÷√


        private void Start()
        {
            m_meshFilter = GetComponentInChildren<MeshFilter>();
            Mesh mesh = m_meshFilter.mesh;
            m_mesh = m_meshFilter.mesh;
            m_initL2W = this.transform.localToWorldMatrix;
            if (m_dataSourceType == DataSourceType.Mesh)
            {
                var len = mesh.vertices.Length;
                rest_X = new Vector3[len];
                m_x = new Vector3[len];
                for (int i = 0; i < len; i++)
                {
                    var p = m_initL2W.MultiplyPoint3x4(mesh.vertices[i]);
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
                    var p = m_initL2W.MultiplyPoint3x4(m_tetMesh.GetParticlePos(i));
                    m_x[i] = p;
                    rest_X[i] = p;
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

        public override float GetParticleStaticFriction(int particleIndex)
        {
            return m_staticFriction;
        }

        public override float GetParticleDynamicFriction(int particleIndex)
        {
            return m_dynamicFriction;
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
            this.transform.localPosition = Vector3.zero;
            this.transform.localRotation = Quaternion.identity;
            this.transform.localScale = Vector3.one;
        }
    }
}
