using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    [CustomEditor(typeof(BunnyTetMeshImpl))]
    public class TetMeshInspector : Editor
    {
        private SerializedObject m_editTarget;
        private TetMesh m_tetMesh;

        public virtual void OnEnable()
        {
            m_tetMesh = target as TetMesh;
            m_editTarget = new SerializedObject(m_tetMesh);
        }

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();
            if (GUILayout.Button("��ʼ��"))
            {
                m_tetMesh.Init();
            }
            if (GUILayout.Button("ͬ��Mesh"))
            {
                m_tetMesh.Sync2Mesh4Editor();
            }
        }
    }
}
