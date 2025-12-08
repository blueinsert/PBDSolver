using Unity.Collections;
using Unity.Mathematics;

namespace bluebean.Physics.PBD.DataStruct
{
    public struct BurstTriangleMesh : IDistanceFunction
    {
        public BurstColliderShape shape;
        public BurstAffineTransform colliderToWorld;

        public TriangleMeshHeader header;
        public NativeArray<BIHNode> bihNodes;
        public NativeArray<Triangle> triangles;
        public NativeArray<float3> vertices;

        public float dt;
        public float collisionMargin;

        private BurstMath.CachedTri tri;

        /// <summary>
        /// 在mesh的本地坐标系中计算最近点
        /// </summary>
        /// <param name="point"></param>
        /// <param name="radii"></param>
        /// <param name="projectedPoint"></param>
        public void Evaluate(float4 point, float4 radii, quaternion quaternion, ref SurfacePoint projectedPoint)
        {
            //从世界坐标系转碰撞体本地坐标系
            point = colliderToWorld.InverseTransformPoint(point);
            //point = colliderToSolver.InverseTransformPointUnscaled(point);
            //计算距离当前tri的最近点
            float4 nearestPoint = BurstMath.NearestPointOnTri(tri, point, out float4 bary);
            float4 normal = math.normalizesafe(point - nearestPoint);

            //转回世界坐标系
            projectedPoint.point = colliderToWorld.TransformPoint(nearestPoint + normal * shape.contactOffset);
            projectedPoint.normal = colliderToWorld.TransformDirection(normal);
        }

        public void Contacts(int colliderIndex,
                              //int rigidbodyIndex,
                              // NativeArray<BurstRigidbody> rigidbodies,
                              NativeArray<float4> positions,
                              NativeArray<float4> velocities,
                              NativeArray<float4> radii,
                              in BurstAabb particleBounds,
                              int particleIndex,

                              NativeQueue<BurstContact>.ParallelWriter contacts
                              )
        {

            BIHTraverse(colliderIndex, particleIndex,
                        positions, velocities, radii, in particleBounds, 0, contacts);

        }

        private void BIHTraverse(int colliderIndex,
                                 //int rigidbodyIndex,
                                 int particleIndex,
                                 //NativeArray<BurstRigidbody> rigidbodies,
                                 NativeArray<float4> positions,
                                 NativeArray<float4> velocities,
                                 NativeArray<float4> radii,
                                 in BurstAabb particleBounds,
                                 int nodeIndex,
                                 NativeQueue<BurstContact>.ParallelWriter contacts
            )
        {
            var node = bihNodes[header.firstNode + nodeIndex];

            if (node.firstChild >= 0)
            {
                // visit min node:
                if (particleBounds.min[node.axis] <= node.leftSplitPlane)
                    BIHTraverse(colliderIndex, particleIndex,
                                positions, velocities, radii, in particleBounds,
                                node.firstChild, contacts);

                // visit max node:
                if (particleBounds.max[node.axis] >= node.rightSplitPlane)
                    BIHTraverse(colliderIndex, particleIndex,
                                positions, velocities, radii, in particleBounds,
                                node.firstChild + 1, contacts);
            }
            else
            {
                //已经是叶子节点，将叶子节点包含的所有三角形与粒子做蛮力碰撞检测
                // check for contact against all triangles:
                for (int dataOffset = node.start; dataOffset < node.start + node.count; ++dataOffset)
                {
                    Triangle t = triangles[header.firstTriangle + dataOffset];
                    float4 v1 = new float4(vertices[header.firstVertex + t.i1], 0);
                    float4 v2 = new float4(vertices[header.firstVertex + t.i2], 0);
                    float4 v3 = new float4(vertices[header.firstVertex + t.i3], 0);
                    BurstAabb triangleBounds = new BurstAabb(v1, v2, v3, shape.contactOffset + collisionMargin);

                    //先判断aabb是否相交，再判断顶点级别
                    if (triangleBounds.IntersectsAabb(particleBounds))
                    {
                        tri.Cache(v1, v2, v3);

                        float4 particlePoint = positions[particleIndex];
                        float4 particleVelocity = velocities[particleIndex];
                        float particleRadius = radii[particleIndex].x;

                        var nearestPoint = new SurfacePoint();
                        this.Evaluate(particlePoint, particleRadius, quaternion.identity, ref nearestPoint);

                        float4 rbVelocity = float4.zero;  
                        //if (rigidbodyIndex >= 0)
                        //   rbVelocity = BurstMath.GetRigidbodyVelocityAtPoint(rigidbodyIndex, colliderPoint.point, rigidbodies, solverToWorld);
                        //计算粒子点距离表面最近点的相对距离和相对速度
                        float dAB = math.dot(particlePoint - nearestPoint.point, nearestPoint.normal);
                        float dVel = math.dot(particleVelocity - rbVelocity, nearestPoint.normal);
                        //判断在这一帧内是否会碰撞
                        if (dVel * dt + dAB <= particleRadius + shape.contactOffset + collisionMargin)
                        {
                            contacts.Enqueue(new BurstContact()
                            {
                                bodyA = particleIndex,
                                bodyB = colliderIndex,
                                pointA = new float4(1, 0, 0, 0),
                                pointB = nearestPoint.point,
                                normal = nearestPoint.normal,
                                distance = dAB,
                            });
                        }
                    }
                }
            }
        }

    }

}
