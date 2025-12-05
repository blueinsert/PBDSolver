using bluebean.Physics.PBD.DataStruct;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    /// <summary>
    /// 四面体网格
    /// </summary>
    [RequireComponent(typeof(MeshFilter))]
    public class TetMesh : MonoBehaviour
    {
        //粒子，也就是顶点
        public int m_numParticles = 0;
        //边的数量
        public int m_numEdges = 0;
        //四面体的数量
        public int m_numTets = 0;
        //表面的三角形数量（一个mesh可能有多个表面）
        public int m_numSurfs = 0;
        //当前位置或预测位置数组
        public Vector3[] m_pos = null;
        //四面体的顶点索引数组，每一个元素代表一个四面体，四个整数代表四个顶点
        public VectorInt4[] m_tet = null;
        //每个元素代表一条边，两个整数代表两个端点的索引
        public Vector2Int[] m_edge = null;
        //每个元素代表一个表面，三个整数代表三个顶点的索引
        public Vector3Int[] m_surf = null;
        //表面三角形的索引
        public int[] m_tetSurfaceTriIds;
        //四面体的初始体积
        public float[] m_restVol = null;
        //边的初始长度
        public float[] m_restLen = null;
        //每个粒子的初始质量
        public float[] m_mass = null;
        //每个粒子的初始质量的倒数
        public float[] m_invMass = null;
        //粒子的颜色数组（用于固定点）
        public Color[] m_particleColors = null;

        public bool m_isInitialized = false;

        public virtual void Init() { }
        public virtual string GetMeshName()
        {
            return "NewMesh";
        }

        public static float CalcTetVolume(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4)
        {
            var temp = Vector3.Cross(p2 - p1, p3 - p1);
            float res = Vector3.Dot(p4 - p1, temp);
            res *= (1.0f / 6);
            if (res < -1E-06)
            {
                //Debug.LogError(string.Format("volume < 0,{0}", res));
            }
            return res;
        }

        /// <summary>
        /// 获取四面体的体积
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public float TetVolume(int index)
        {
            var ids = m_tet[index];
            var id1 = ids.x;
            var id2 = ids.y;
            var id3 = ids.z;
            var id4 = ids.w;
            var p1 = m_pos[id1];
            var p2 = m_pos[id2];
            var p3 = m_pos[id3];
            var p4 = m_pos[id4];
            var res = CalcTetVolume(p1, p2, p3, p4);
            return res;
        }

        public void Sync2Mesh4Editor()
        {
            if (!m_isInitialized) return;
            var meshFilter = this.GetComponent<MeshFilter>();
            if (meshFilter != null)
            {
                var oldMesh = meshFilter.sharedMesh;
                var mesh = new Mesh();
                mesh.name = GetMeshName();
                mesh.vertices = this.m_pos;
                mesh.triangles = this.m_tetSurfaceTriIds;
                mesh.RecalculateNormals();
                mesh.RecalculateBounds();
                meshFilter.sharedMesh = mesh;
                if (oldMesh != null)
                {
                    if (oldMesh.vertices.Length == m_pos.Length)
                    {
                        m_particleColors = new Color[oldMesh.colors.Length];
                        for (int i = 0; i < m_particleColors.Length; i++)
                        {
                            m_particleColors[i] = oldMesh.colors[i];
                        }
                    }
                    mesh.colors = m_particleColors;
                }

            }
        }

        public float GetEdgeRestLen(int edgeIndex)
        {
            return m_restLen[edgeIndex];
        }


        public float GetParticleInvMass(int particleIndex)
        {
            return m_invMass[particleIndex];
        }

        public float GetTetRestVolume(int tetIndex)
        {
            return m_restVol[tetIndex];
        }

        public VectorInt4 GetTetVertexIndex(int tetIndex)
        {
            var tet = m_tet[tetIndex];
            VectorInt4 res = new VectorInt4((int)tet.x, (int)tet.y, (int)tet.z, (int)tet.w);
            return res;
        }

        public Vector2Int GetEdgeParticles(int edgeIndex)
        {
            return m_edge[edgeIndex];
        }

        public bool IsParticleFixed(int particleIndex)
        {
            if (particleIndex == 25)
                return true;
            return false;
            if (particleIndex < 0 || particleIndex >= m_particleColors.Length)
            {
                return false;
            }
            return m_particleColors[particleIndex].r > 0;
        }

        public Vector3 GetParticlePos(int particleIndex)
        {
            return m_pos[particleIndex];
        }
    }
}