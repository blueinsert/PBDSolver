using bluebean.Physics.PBD.DataStruct;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    public partial class PBDSolver
    {

        public class CollisionEventArgs : System.EventArgs
        {
            public ObiList<Contact> m_contacts = new ObiList<Contact>();
        }

        public delegate void CollisionCallback(PBDSolver solver, CollisionEventArgs contacts);

        private ColliderWorld m_colliderWorld = new ColliderWorld();
        public NativeArray<BurstContact> m_colliderContacts;

        private CollisionEventArgs m_collisionArgs = new CollisionEventArgs();
        public event CollisionCallback EventOnCollision;

        /// <summary>
        /// ���������ٶȣ�deltaTime�ȸ�����һ֡�����ӵĿ��ܻ��Χ
        /// </summary>
        /// <param name="deltaTime"></param>
        /// <returns></returns>
        private JobHandle UpdateParticleBounds(float deltaTime)
        {
            var buildAabbs = new BuildParticleAabbsJob
            {
                //����
                radii = this.ParticleRadius,
                positions = this.ParticlePositions,
                velocities = this.ParticleVels,
                collisionMargin = 0.01f,
                continuousCollisionDetection = 1,
                dt = deltaTime,
                //���
                simplexBounds = this.ParticleAabb,
            };
            return buildAabbs.Schedule(this.ParticlePositions.Count(), 32);
        }

        /// <summary>
        /// ��m_colliderContacts�������ݵ�contacts
        /// </summary>
        /// <param name="contacts"></param>
        /// <param name="count"></param>
        private void GetCollisionContacts(Contact[] contacts, int count)
        {
            NativeArray<Contact>.Copy(m_colliderContacts.Reinterpret<Contact>(), 0, contacts, 0, count);
        }

        private void CollisionDetection(float deltaTime)
        {
            //�������ӵ�ǰ֡���aabb
            var updateSimplexBoundsHandle = UpdateParticleBounds(deltaTime);
            //�������Ӻͻ�����ײ�����ײ�Ӵ�����
            var gemterateCpmtactsHandle = m_colliderWorld.GenerateContacts(deltaTime, updateSimplexBoundsHandle);
            gemterateCpmtactsHandle.Complete();

            m_colliderContacts = new NativeArray<BurstContact>(m_colliderWorld.m_colliderContactQueue.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            //����ײ�Ӵ����ݴ�collideWorld��ȡ�����ر���m_colliderContacts��
            DequeueIntoArrayJob<BurstContact> dequeueColliderContacts = new DequeueIntoArrayJob<BurstContact>()
            {
                InputQueue = m_colliderWorld.m_colliderContactQueue,
                OutputArray = m_colliderContacts
            };
            dequeueColliderContacts.Schedule().Complete();
            if (m_colliderContacts.Length > 0)
            {
                Debug.Log($"contacts count: {m_colliderContacts.Length}");
            }
        }
    }
}
