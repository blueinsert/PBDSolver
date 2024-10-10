using bluebean.Physics.PBD.DataStruct;
using bluebean.Physics.PBD.DataStruct.Native;
using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using UnityEngine.UIElements;

namespace bluebean.Physics.PBD
{
    public partial class PBDSolver : MonoBehaviour, ISolver
    {
        public ColliderWorld ColliderWorld
        {
            get
            {
                return m_colliderWorld;
            }
        }

        #region 内部变量

        public int m_targetFrameRate = 60;
        public float m_dtSubStep = 0.0333f;
        public float m_dtStep = 0.0333f;
        public float m_damping = 0.99f;
        public float m_damping_subStep = 0.99f;
        [Range(0f, 0.002f)]
        public float m_edgeCompliance = 0.0f;
        [Range(0f, 1f)]
        public float m_volumeCompliance = 0.0f;
        [Range(1, 55)]
        public int m_subStep = 22;
        [Range(0f, 1f)]
        public float m_collideCompliance = 0.0f;
        [Header("重力加速度")]
        public Vector3 m_g = new Vector3(0, -9.8f, 0);

        public List<PDBActor> m_actors = new List<PDBActor>();
        private Dictionary<int, PDBActor> m_actorDic = new Dictionary<int, PDBActor>();

        private Dictionary<int, ConstrainGroup> m_constrains = new Dictionary<int, ConstrainGroup>();

        #region 粒子数据

        public NativeVector4List PositionList { get { return m_positionList; } }
        public NativeVector4List VelList { get { return m_velList; } }
        public NativeVector4List ExternalForceList
        {
            get { return m_externalForceList; }
            set { m_externalForceList = value; }
        }
        public NativeFloatList InvMassList { get { return m_invMassList; } }

        // all particle positions [NonSerialized] private
        private NativeVector4List m_positionList = new NativeVector4List();
        private NativeVector4List m_radiusList = new NativeVector4List();
        private NativeVector4List m_prevPositionList = new NativeVector4List();
        private NativeVector4List m_velList = new NativeVector4List();
        private NativeVector4List m_propertyList = new NativeVector4List();
        private NativeAabbList m_aabbList = new NativeAabbList();
        private NativeVector4List m_externalForceList = new NativeVector4List();
        private NativeIntList m_freeList = new NativeIntList();
        private NativeFloatList m_invMassList = new NativeFloatList();
        private NativeVector4List m_positionDeltaList = new NativeVector4List();
        private NativeVector4List m_gradientList = new NativeVector4List();
        private NativeIntList m_positionConstraintCountList = new NativeIntList();

        private NativeArray<float4> m_particlePositions;
        private NativeArray<float4> m_prevParticlePositions;
        private NativeArray<float4> m_particleVels;
        private NativeArray<float4> m_particleProperties;
        private NativeArray<float4> m_particleRadius;
        private NativeArray<BurstAabb> m_particleAabbs;
        private NativeArray<float4> m_externalForces;
        private NativeArray<float> m_invMasses;
        private NativeArray<float4> m_positionDeltas;
        private NativeArray<float4> m_gradients;
        private NativeArray<int> m_positionConstraintCounts;

        public NativeArray<float4> ParticlePositions => m_particlePositions;
        public NativeArray<float> InvMasses => m_invMasses;
        public NativeArray<float4> PositionDeltas => m_positionDeltas;
        public NativeArray<float4> Gradients => m_gradients;
        public NativeArray<int> PositionConstraintCounts => m_positionConstraintCounts;
        public NativeArray<float4> ParticleVels => m_particleVels;
        public NativeArray<float4> ExternalForces => m_externalForces;
        public NativeArray<float4> ParticleProperties => m_particleProperties;

        public NativeArray<float4> PrevParticlePositions => m_prevParticlePositions;

        public float StretchConstrainCompliance => m_edgeCompliance;

        public float VolumeConstrainCompliance => m_volumeCompliance;

        public NativeArray<float4> ParticleRadius => m_particleRadius;

        public NativeArray<BurstAabb> ParticleAabb => m_particleAabbs;

        public NativeArray<BurstContact> ColliderContacts => m_colliderContacts;
        #endregion

        #endregion

        #region 生命周期

        public void Awake()
        {
            Application.targetFrameRate = m_targetFrameRate;
            m_dtStep = 1.0f / m_targetFrameRate;
            m_damping_subStep = Mathf.Pow(m_damping, 1.0f / m_subStep);
            m_dtSubStep = m_dtStep / m_subStep;
            Intialize();

        }

        private void Intialize()
        {
            m_colliderWorld = new ColliderWorld();
            m_colliderWorld.Initialzie(this);
            InitConstrains();
        }

        void OnDestroy()
        {
            Debug.Log("PBDSolver:OnDestroy");
            m_positionList.Dispose();
            m_velList.Dispose();
            m_propertyList.Dispose();
            m_externalForceList.Dispose();
            m_freeList.Dispose();
            m_invMassList.Dispose();
            m_positionDeltaList.Dispose();
            m_gradientList.Dispose();
            m_positionConstraintCountList.Dispose();
            m_radiusList.Dispose();
            m_aabbList.Dispose();

            m_colliderWorld.Destroy();
        }

        void OnPreStep()
        {
            //碰撞侦测在step之前做一次，不在substep中进行
            //之后在每个substep中会做碰撞求解
            m_colliderWorld.UpdateWorld(m_dtStep);
            CollisionDetection(m_dtStep);

            for (int i = 0; i < m_actors.Count; i++)
            {
                m_actors[i].OnPreStep();
            }
        }

        void OnPostStep()
        {
            for (int i = 0; i < m_actors.Count; i++)
            {
                m_actors[i].OnPostStep();
            }

            ClearForce();

            //对外抛出碰撞事件，传出碰撞信息，用于调试绘图等
            if (EventOnCollision != null)
            {
                var contactCount = this.m_colliderContacts.Length;
                m_collisionArgs.m_contacts.SetCount(contactCount);

                if (contactCount > 0)
                    GetCollisionContacts(m_collisionArgs.m_contacts.Data, contactCount);

                EventOnCollision(this, m_collisionArgs);
            }

            if (m_colliderContacts.IsCreated)
                m_colliderContacts.Dispose();
        }

        void OnPreSubStep()
        {
            for (int i = 0; i < m_actors.Count; i++)
            {
                m_actors[i].OnPreSubStep(m_dtSubStep, m_g);
            }
        }

        void OnPostSubStep()
        {
            for (int i = 0; i < m_actors.Count; i++)
            {
                m_actors[i].OnPostSubStep(m_dtSubStep, m_damping_subStep);
            }
        }

        void SubStep(float stepTime, float substepTime, int substeps)
        {
            Profiler.BeginSample("SubStep");
            JobHandle handle = new JobHandle();
            //1.预测位置
            PredictPositionsJob predictPositionsJob = new PredictPositionsJob()
            {
                m_deltaTime = substepTime,
                m_gravity = new float4(m_g.x, m_g.y, m_g.z, 0),
                m_positions = ParticlePositions,
                m_prevPositions = this.PrevParticlePositions,
                m_externalForces = ExternalForces,
                m_velocities = ParticleVels,
                m_inverseMasses = InvMasses,
                m_particleProperties = this.ParticleProperties,
            };
            handle = predictPositionsJob.Schedule(ParticlePositions.Count(), 4, handle);

            //2.求解约束
            var start = (int)ConstrainType.Start + 1;
            var end = (int)ConstrainType.Max;
            for (int i = start; i < end; i++)
            {
                var constrain = m_constrains[i];
                handle = constrain.Solve(handle, stepTime, substepTime, substeps);
                handle = constrain.Apply(handle, substepTime);
            }
            //3.更新速度
            var updateVel = new UpdateVelJob()
            {
                m_deltaTime = substepTime,
                m_positions = this.ParticlePositions,
                m_prevPositions = this.PrevParticlePositions,
                m_velocities = this.ParticleVels,
                m_velDamping = this.m_damping_subStep,
                m_sleepThreshold = 0.00001f,
            };
            handle = updateVel.Schedule(m_positionList.count, 32, handle);

            handle.Complete();
            Profiler.EndSample();
        }


        // Update is called once per frame
        void FixedUpdate()
        {
            Profiler.BeginSample("PreStep");
            OnPreStep();
            Profiler.EndSample();
            Profiler.BeginSample("SubStep");
            for (int i = 0; i < m_subStep; i++)
            {
                Profiler.BeginSample("PreSubStep");
                OnPreSubStep();
                Profiler.EndSample();

                SubStep(m_dtStep, m_dtSubStep, m_subStep - i);

                Profiler.BeginSample("PostSubStep");
                OnPostSubStep();
                Profiler.EndSample();
            }
            Profiler.EndSample();
            Profiler.BeginSample("PreStep");
            OnPostStep();
            Profiler.EndSample();
        }

        #endregion

        #region 内部方法

        private void ClearForce()
        {
            m_externalForceList.WipeToZero();
        }

        private void InitConstrains()
        {
            m_constrains[(int)ConstrainType.Collide] = new CollideConstrainGroup(this);
            m_constrains[(int)ConstrainType.Stretch] = new StretchConstrainGroup(this);
            m_constrains[(int)ConstrainType.Volume] = new VolumeConstrainGroup(this);
        }

        private void OnParticleCountChange()
        {
            //重新从nativeList中获取nativeArray引用
            m_particlePositions = m_positionList.AsNativeArray<float4>();
            m_particleVels = m_velList.AsNativeArray<float4>();
            m_externalForces = m_externalForceList.AsNativeArray<float4>();
            m_invMasses = m_invMassList.AsNativeArray<float>();
            m_positionDeltas = m_positionDeltaList.AsNativeArray<float4>();
            m_gradients = m_gradientList.AsNativeArray<float4>();
            m_positionConstraintCounts = m_positionConstraintCountList.AsNativeArray<int>();
            m_particleProperties = m_propertyList.AsNativeArray<float4>();
            m_prevParticlePositions = m_prevPositionList.AsNativeArray<float4>();
            m_particleRadius = m_radiusList.AsNativeArray<float4>();
            m_particleAabbs = m_aabbList.AsNativeArray<BurstAabb>();
        }

        private void EnsureParticleArraysCapacity(int count)
        {
            // only resize if the count is larger than the current amount of particles:
            if (count >= m_positionList.count)
            {
                m_positionList.ResizeInitialized(count);
                m_velList.ResizeInitialized(count);
                m_externalForceList.ResizeInitialized(count);
                m_invMassList.ResizeInitialized(count);
                m_positionDeltaList.ResizeInitialized(count);
                m_gradientList.ResizeInitialized(count);
                m_positionConstraintCountList.ResizeInitialized(count);
                m_propertyList.ResizeInitialized(count);
                m_prevPositionList.ResizeInitialized(count);
                m_aabbList.ResizeInitialized(count);
                m_radiusList.ResizeInitialized(count);

                OnParticleCountChange();
            }
        }

        private void AllocateParticles(int[] particleIndices)
        {

            // If attempting to allocate more particles than we have:
            if (particleIndices.Length > m_freeList.count)
            {
                int grow = particleIndices.Length - m_freeList.count;

                // append new free indices:
                for (int i = 0; i < grow; ++i)
                    m_freeList.Add(m_positionList.count + i);

                // grow particle arrays:
                EnsureParticleArraysCapacity(m_positionList.count + particleIndices.Length);
            }

            // determine first particle in the free list to use:
            int first = m_freeList.count - particleIndices.Length;

            // copy free indices to the input array:
            m_freeList.CopyTo(particleIndices, first, particleIndices.Length);

            // shorten the free list:
            m_freeList.ResizeUninitialized(first);

        }

        #endregion

        #region 对外接口
        public void AddActor(PDBActor actor)
        {
            if (!m_actorDic.ContainsKey(actor.ActorId))
            {
                m_actorDic.Add(actor.ActorId, actor);
                m_actors.Add(actor);

                //初始化actor的粒子索引数组
                actor.m_particleIndicesInSolver = new int[actor.GetParticleCount()];
                AllocateParticles(actor.m_particleIndicesInSolver);

                //初始化这个actor的所有粒子的数据
                for (int i = 0; i < actor.GetParticleCount(); i++)
                {
                    var index = actor.m_particleIndicesInSolver[i];//在求解器中的索引
                    var pos = actor.GetParticleInitPosition(i);
                    this.m_positionList[index] = pos;
                    this.m_externalForceList[index] = Vector4.zero;
                    this.m_velList[index] = Vector4.zero;
                    this.m_invMassList[index] = actor.GetParticleInvMass(i);
                    this.m_positionDeltaList[index] = Vector4.zero;
                    this.m_gradientList[index] = Vector4.zero;
                    this.m_positionConstraintCountList[index] = 0;
                    float radius = 0.1f;
                    this.m_radiusList[index] = new Vector4(radius, radius, radius, radius);
                    this.m_aabbList[index] = new Aabb();
                }
                Debug.Log("AddActor Finish");
            }
        }

        public void RemoveActor(PDBActor actor)
        {
            m_actors.Remove(actor);
            m_actorDic.Remove(actor.ActorId);
        }

        public void PushStretchConstrain(StretchConstrainData stretchConstrainData)
        {
            var constrainGroup = m_constrains[(int)ConstrainType.Stretch] as StretchConstrainGroup;
            var actorId = stretchConstrainData.m_actorId;
            var actor = m_actorDic[actorId];
            var p1 = actor.m_particleIndicesInSolver[stretchConstrainData.m_edge.x];
            var p2 = actor.m_particleIndicesInSolver[stretchConstrainData.m_edge.y];
            constrainGroup.AddConstrain(new VectorInt2(p1, p2), stretchConstrainData.m_restLen, 0);
        }

        public void PushVolumeConstrain(VolumeConstrainData volumeConstrainData)
        {
            var constrainGroup = m_constrains[(int)ConstrainType.Volume] as VolumeConstrainGroup;
            var actorId = volumeConstrainData.m_actorId;
            var actor = m_actorDic[actorId];
            var p1 = actor.m_particleIndicesInSolver[volumeConstrainData.m_tet.x];
            var p2 = actor.m_particleIndicesInSolver[volumeConstrainData.m_tet.y];
            var p3 = actor.m_particleIndicesInSolver[volumeConstrainData.m_tet.z];
            var p4 = actor.m_particleIndicesInSolver[volumeConstrainData.m_tet.w];
            constrainGroup.AddConstrain(new VectorInt4(p1, p2, p3, p4), volumeConstrainData.m_restVolume, 0);
        }

        public Vector3 GetParticlePosition(int particleIndex)
        {
            var pos = m_positionList[particleIndex];
            return pos;
        }

        #endregion
    }
}
