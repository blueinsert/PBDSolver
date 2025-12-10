using bluebean.Physics.PBD.DataStruct;
using System.Linq;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEditor.Rendering.Universal.ShaderGUI;
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

        private ColliderWorld m_colliderWorld;
        public NativeArray<BurstContact> m_colliderContacts;

        private CollisionEventArgs m_collisionArgs = new CollisionEventArgs();
        public event CollisionCallback EventOnCollision;

        private ParticleGrid m_particleGrid;
        public NativeArray<BurstContact> m_particleContacts;
        public NativeArray<BatchData> m_particleBatchData;
        private ContactBatcher m_particleContactBatcher;

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

        /// <summary>
        /// 产生粒子与unity碰撞体直接的碰撞数据
        /// </summary>
        /// <param name="deltaTime"></param>
        /// <param name="deps"></param>
        private void GenerateParticleColliderContacts(float deltaTime, JobHandle deps)
        {
            //产生粒子和环境碰撞体的碰撞接触数据
            var gemterateCpmtactsHandle = m_colliderWorld.GenerateContacts(deltaTime, deps);
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
                Debug.Log($"collider contacts count: {m_colliderContacts.Length}");
            }
        }

        public JobHandle SortParticleContacts(int particleCount,
                                         NativeArray<BurstContact> constraints,
                                         ref NativeArray<BurstContact> sortedConstraints,
                                         JobHandle handle)
        {
            // Count the amount of digits in the largest particle index that can be referenced by a constraint:
            NativeArray<int> totalCountUpToDigit = new NativeArray<int>(particleCount + 1, Allocator.TempJob);
            int numDigits = 0;
            int maxBodyIndex = particleCount - 1;
            {
                int val = maxBodyIndex;
                while (val > 0)
                {
                    val >>= 1;
                    numDigits++;
                }
            }

            handle = new CountSortPerFirstParticleJob
            {
                input = constraints,
                output = sortedConstraints,
                maxDigits = numDigits,
                maxIndex = maxBodyIndex,
                digitCount = totalCountUpToDigit
            }.Schedule(handle);

            // Sort sub arrays with default sort.
            int numPerBatch = math.max(1, maxBodyIndex / 32);

            handle = new SortSubArraysJob
            {
                InOutArray = sortedConstraints,
                NextElementIndex = totalCountUpToDigit,
                comparer = new BurstContactComparer(),
            }.Schedule(totalCountUpToDigit.Length, numPerBatch, handle);

            return handle;
        }

        private void GenerateParticleParticleContacts(float deltaTime, JobHandle inputDeps)
        {
            int particleCount = GetParticleCount();

            m_particleGrid.Update(this, deltaTime, inputDeps);
            var generateParticleInteractionsHandle = m_particleGrid.GenerateContacts(this, deltaTime);
            generateParticleInteractionsHandle.Complete();

            // allocate arrays for interactions and batch data:
            m_particleContacts = new NativeArray<BurstContact>(m_particleGrid.m_particleContactQueue.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            m_particleBatchData = new NativeArray<BatchData>(MaxBatches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var rawParticleContacts = new NativeArray<BurstContact>(m_particleGrid.m_particleContactQueue.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var sortedParticleContacts = new NativeArray<BurstContact>(m_particleGrid.m_particleContactQueue.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            DequeueIntoArrayJob<BurstContact> dequeueParticleContacts = new DequeueIntoArrayJob<BurstContact>()
            {
                InputQueue = m_particleGrid.m_particleContactQueue,
                OutputArray = rawParticleContacts
            };
            var dequeueHandle = dequeueParticleContacts.Schedule();
            // Sort contacts for jitter-free gauss-seidel (sequential) solving:
            // 根据约束的第一个及第二个粒子编号，排序约束数组
             dequeueHandle = SortParticleContacts(GetParticleCount(), rawParticleContacts, ref sortedParticleContacts, dequeueHandle);

            ContactProvider contactProvider = new ContactProvider()
            {
                contacts = sortedParticleContacts,
                sortedContacts = m_particleContacts,
            };
            var activeParticleBatchCount = new NativeArray<int>(1, Allocator.TempJob);
            var particleBatchHandle = m_particleContactBatcher.BatchConstraints(ref contactProvider, particleCount, ref m_particleBatchData, ref activeParticleBatchCount, dequeueHandle);
            particleBatchHandle.Complete();

            if (m_particleContacts.Length > 0)
            {
                Debug.Log($"particle contacts count: {m_particleContacts.Length}");
            }

            rawParticleContacts.Dispose();
            sortedParticleContacts.Dispose();
            activeParticleBatchCount.Dispose();
        }

        private void CollisionDetection(float deltaTime)
        {
            //更新粒子当前帧活动的aabb
            var updateParticleBoundsHandle = UpdateParticleBounds(deltaTime);

            GenerateParticleColliderContacts(deltaTime, updateParticleBoundsHandle);

            GenerateParticleParticleContacts(deltaTime, updateParticleBoundsHandle);
        }
    }
}
