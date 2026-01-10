using bluebean.Physics.PBD;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(PBDSolver))]
public class SolverDebugInfoGUI : MonoBehaviour
{
    GUIStyle m_style;
    PBDSolver m_solver = null;

    private void Awake()
    {
        m_solver = GetComponent<PBDSolver>();
    }

    // Use this for initialization  
    void Start()
    {
        m_style = new GUIStyle();
        m_style.fontSize = 30;       //字体大小
    }

    // Update is called once per frame  
    void Update()
    {
    }

    void OnGUI()
    {
        if (m_solver == null)
            return;
        if (!m_solver.m_showGUI) return;
        // 显示粒子数量
        GUILayout.Label($"当前粒子数: {m_solver.PositionList.count}",m_style);
        GUILayout.Label($"contact: {m_solver.m_colliderContacts.Length}", m_style);
        GUILayout.Label($"particle contact: {m_solver.m_particleContacts.Length}", m_style);
    }

}
