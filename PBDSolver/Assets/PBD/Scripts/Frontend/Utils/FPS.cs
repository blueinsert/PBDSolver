using System.Collections;
using System.Collections.Generic;
#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine;

public class FPS : MonoBehaviour
{
    private void Awake()
    {
        Application.targetFrameRate = 60;

    }

    void ComputeFps()
    {
        ++m_updateCount;
        float now = Time.realtimeSinceStartup;
        float dt = now - m_lastFpsResetTime;
        if (dt > 0.25f)
        {
            m_fps = m_updateCount / dt;
            m_updateCount = 0;
            m_lastFpsResetTime = now;
        }
    }

    void Update()
    {
        ComputeFps();

        if (Input.GetKeyUp(KeyCode.Escape))
        {
#if UNITY_EDITOR
            EditorApplication.isPaused = !EditorApplication.isPaused;
#endif
        }
    }

    void InitStyle(int unit)
    {
        m_unit = unit;
        m_buttonStyle = new GUIStyle(GUI.skin.button);
        m_buttonStyle.fontSize = unit * 7 / 8;
        m_buttonStyle.padding = new RectOffset();

        m_textFieldStyle = new GUIStyle(GUI.skin.textField);
        m_textFieldStyle.fontSize = unit * 7 / 8;
        m_textFieldStyle.padding = new RectOffset();

        m_textStyle = new GUIStyle(GUI.skin.label);
        m_textStyle.normal.textColor = Color.white;
        m_textStyle.fontSize = unit * 7 / 8;
        m_textStyle.padding = new RectOffset();

        m_textStyleGray = new GUIStyle(GUI.skin.label);
        m_textStyleGray.normal.textColor = new Color(0.7f, 0.7f, 0.7f);
        m_textStyleGray.fontSize = unit * 7 / 8;
        m_textStyleGray.padding = new RectOffset();

        m_textStyleSmall = new GUIStyle(GUI.skin.label);
        m_textStyleSmall.normal.textColor = Color.white;
        m_textStyleSmall.fontSize = unit * 7 / 10;
        m_textStyleSmall.padding = new RectOffset();

        m_textStyleSmallGray = new GUIStyle(GUI.skin.label);
        m_textStyleSmallGray.normal.textColor = new Color(0.7f, 0.7f, 0.7f);
        m_textStyleSmallGray.fontSize = unit * 7 / 10;
        m_textStyleSmallGray.padding = new RectOffset();

        m_textStyleSmallCenter = new GUIStyle(GUI.skin.label);
        m_textStyleSmallCenter.normal.textColor = Color.white;
        m_textStyleSmallCenter.fontSize = unit * 7 / 10;
        m_textStyleSmallCenter.padding = new RectOffset();
        m_textStyleSmallCenter.alignment = TextAnchor.UpperCenter;

        m_textStyleSmall2 = new GUIStyle(GUI.skin.label);
        m_textStyleSmall2.normal.textColor = Color.white;
        m_textStyleSmall2.fontSize = unit * 5 / 10;
        m_textStyleSmall2.padding = new RectOffset();
    }

    void OnGUI()
    {
        int unit = Screen.width / 80;
        if (unit != m_unit)
        {
            InitStyle(unit);
        }

        int x = 100;
        int y = Screen.height - m_unit-10;
        GUI.Label(new Rect(x, y, m_unit * 10, m_unit * 2), string.Format("FPS  {0}", m_fps), m_textStyle);
   
    }


    int m_updateCount;
    float m_fps;
    float m_lastFpsResetTime;

    int m_unit = 0;

    GUIStyle m_buttonStyle = null;
    GUIStyle m_textFieldStyle = null;
    GUIStyle m_textStyle = null;
    GUIStyle m_textStyleGray = null;
    GUIStyle m_textStyleSmall = null;
    GUIStyle m_textStyleSmallGray = null;
    GUIStyle m_textStyleSmallCenter = null;
    GUIStyle m_textStyleSmall2 = null;

}
