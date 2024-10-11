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
        /// 根据粒子速度，deltaTime等更新这一帧内粒子的可能活动范围
        /// </summary>
        /// <param name="deltaTime"></param>
        /// <returns></returns>
        private JobHandle UpdateParticleBounds(float deltaTime)
        {
            var buildAabbs = new BuildParticleAabbsJob
            {
                //输入
                radii = this.ParticleRadius,
                positions = this.ParticlePositions,
                velocities = this.ParticleVels,
                collisionMargin = 0.01f,
                continuousCollisionDetection = 1,
                dt = deltaTime,
                //输出
                simplexBounds = this.ParticleAabb,
            };
            return buildAabbs.Schedule(this.ParticlePositions.Count(), 32);
        }

        /// <summary>
        /// 从m_colliderContacts复制数据到contacts
        /// </summary>
        /// <param name="contacts"></param>
        /// <param name="count"></param>
        private void GetCollisionContacts(Contact[] contacts, int count)
        {
            NativeArray<Contact>.Copy(m_colliderContacts.Reinterpret<Contact>(), 0, contacts, 0, count);
        }

        private void CollisionDetection(float deltaTime)
        {
            //更新粒子当前帧活动的aabb
            var updateSimplexBoundsHandle = UpdateParticleBounds(deltaTime);
            //产生粒子和环境碰撞体的碰撞接触数据
            var gemterateCpmtactsHandle = m_colliderWorld.GenerateContacts(deltaTime, updateSimplexBoundsHandle);
            gemterateCpmtactsHandle.Complete();

            m_colliderContacts = new NativeArray<BurstContact>(m_colliderWorld.m_colliderContactQueue.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            //将碰撞接触数据从collideWorld提取到本地变量m_colliderContacts中
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
