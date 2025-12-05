using bluebean.Physics.PBD;
using UnityEngine;

[RequireComponent(typeof(PBDSolver))]
public class ContactsDebugDraw : MonoBehaviour
{

    PBDSolver solver;
    public int contactCount;

    PBDSolver.CollisionEventArgs frame;

    void Awake()
    {
        solver = GetComponent<PBDSolver>();
    }

    void OnEnable()
    {
        solver.EventOnCollision += Solver_OnCollision;
    }

    void OnDisable()
    {
        solver.EventOnCollision -= Solver_OnCollision;
    }

    void Solver_OnCollision(object sender, PBDSolver.CollisionEventArgs e)
    {
        frame = e;
    }

    void OnDrawGizmos()
    {
        if (solver == null || frame == null || frame.m_contacts == null) return;

        //Gizmos.matrix = solver.transform.localToWorldMatrix;

        contactCount = frame.m_contacts.Count;

        for (int i = 0; i < frame.m_contacts.Count; ++i)
        {
            var contact = frame.m_contacts.Data[i];

            //if (contact.distance > 0.001f) continue;



            //Gizmos.color = new Color(((i * 100) % 255) / 255.0f, ((i * 50) % 255) / 255.0f, ((i * 20) % 255) / 255.0f);

            int simplexIndex = contact.bodyA;
            var radius = solver.ParticleRadius[simplexIndex].x;
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

}

