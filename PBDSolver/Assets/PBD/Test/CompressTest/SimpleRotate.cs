using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimpleRotate : MonoBehaviour
{
    Quaternion m_initRotate;
    float m_angle;
    public float m_speed = 5.0f;
    // Start is called before the first frame update
    void Start()
    {
        m_initRotate = this.transform.rotation;
    }

    // Update is called once per frame
    void Update()
    {
        m_angle += m_speed * Time.deltaTime;
        var rotate = Quaternion.Euler(0, 0, m_angle) * m_initRotate;
        this.transform.rotation = rotate;
    }
}
