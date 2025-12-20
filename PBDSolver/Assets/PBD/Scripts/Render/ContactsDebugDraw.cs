using bluebean.Physics.PBD;
using UnityEngine;

[RequireComponent(typeof(PBDSolver))]
public class ContactsDebugDraw : MonoBehaviour
{
    PBDSolver solver;
    PBDSolver.CollisionEventArgs m_collideFrameData;
    PBDSolver.CollisionEventArgs m_particleCollideFrameData;

    public int m_contactCount;
    public int m_particleContactCount;
    public bool m_listenColliderContact = true;
    public bool m_listenParticleContact = true;

    void Awake()
    {
        solver = GetComponent<PBDSolver>();
    }

    void OnEnable()
    {
        if(m_listenColliderContact)
            solver.EventOnCollision += Solver_OnCollision;
        if(m_listenParticleContact)
            solver.OnParticleCollision += Solver_OnParticleCollision;
    }

    void OnDisable()
    {
        if (m_listenColliderContact)
            solver.EventOnCollision -= Solver_OnCollision;
        if (m_listenParticleContact)
            solver.OnParticleCollision -= Solver_OnParticleCollision;
    }

    void Solver_OnCollision(object sender, PBDSolver.CollisionEventArgs e)
    {
        m_collideFrameData = e;
    }

    void Solver_OnParticleCollision(object sender, PBDSolver.CollisionEventArgs e)
    {
        m_particleCollideFrameData = e;
    }

    void DrawContacts(PBDSolver.CollisionEventArgs data)
    {
        for (int i = 0; i < data.m_contacts.Count; ++i)
        {
            var contact = data.m_contacts.Data[i];

            int simplexIndex = contact.bodyA;
            var radius = solver.ParticleRadius[simplexIndex];
            var particlePoint = solver.GetParticlePosition(simplexIndex);

            var distance = contact.distance;
            var collidePoint = contact.pointB;
            Vector3 normal = contact.normal;

            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(particlePoint, 0.01f);
            Gizmos.color = (distance <= 0) ? Color.red : Color.green;
            Gizmos.DrawSphere(collidePoint, 0.01f);
            Gizmos.DrawRay(collidePoint, normal.normalized * distance);
        }
    }

    void DrawParticleContacts(PBDSolver.CollisionEventArgs data)
    {
        for (int i = 0; i < data.m_contacts.Count; ++i)
        {
            var contact = data.m_contacts.Data[i];

            int particleA = contact.bodyA;
            var radiusA = 0.1f;// solver.ParticleRadius[particleA];
            var pointA = solver.GetParticlePosition(particleA);

            int particleB = contact.bodyB;
            var radiusB = 0.1f;// solver.ParticleRadius[particleB];
            var pointB = solver.GetParticlePosition(particleB);

            var distance = contact.distance;
            Vector3 normal = contact.normal;

            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(pointA, radiusA);
            Gizmos.color = (distance <= 0) ? Color.red : Color.green;
            Gizmos.DrawSphere(pointB, radiusB);
            Gizmos.DrawRay(pointB, normal.normalized * distance);
        }
    }

    void OnDrawGizmos()
    {
        if (solver == null) return;

        //Gizmos.matrix = solver.transform.localToWorldMatrix;
        if (m_collideFrameData != null)
        {
            m_contactCount = m_collideFrameData.m_contacts.Count;
            DrawContacts(m_collideFrameData);
        }
        if (m_particleCollideFrameData != null)
        {
            m_particleContactCount = m_particleCollideFrameData.m_contacts.Count;
            DrawParticleContacts(m_particleCollideFrameData);
        }
    }

}

