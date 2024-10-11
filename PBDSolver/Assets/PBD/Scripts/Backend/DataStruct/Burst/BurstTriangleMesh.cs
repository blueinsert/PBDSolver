using Unity.Collections;
using Unity.Mathematics;

namespace bluebean.Physics.PBD.DataStruct
{
    public struct BurstTriangleMesh : IDistanceFunction
    {
        public BurstColliderShape shape;
        //public BurstAffineTransform colliderToSolver;
        //public BurstAffineTransform solverToWorld;
        public BurstAffineTransform colliderToWorld;

        public TriangleMeshHeader header;
        public NativeArray<BIHNode> bihNodes;
        public NativeArray<Triangle> triangles;
        public NativeArray<float3> vertices;

        public float dt;
        public float collisionMargin;

        private BurstMath.CachedTri tri;

        /// <summary>
        /// ��mesh�ı�������ϵ�м��������
        /// </summary>
        /// <param name="point"></param>
        /// <param name="radii"></param>
        /// <param name="orientation"></param>
        /// <param name="projectedPoint"></param>
        public void Evaluate(float4 point, float4 radii, quaternion orientation, ref SurfacePoint projectedPoint)
        {
            //����������ϵת��ײ�屾������ϵ
            point = colliderToWorld.InverseTransformPoint(point);
            //point = colliderToSolver.InverseTransformPointUnscaled(point);
            //������뵱ǰtri�������
            float4 nearestPoint = BurstMath.NearestPointOnTri(tri, point, out float4 bary);
            float4 normal = math.normalizesafe(point - nearestPoint);

            //ת����������ϵ
            projectedPoint.point = colliderToWorld.TransformPoint(nearestPoint + normal * shape.contactOffset);// colliderToSolver.TransformPointUnscaled(nearestPoint + normal * shape.contactOffset);
            projectedPoint.normal = colliderToWorld.TransformDirection(normal);// colliderToSolver.TransformDirection(normal);
        }

        public void Contacts(int colliderIndex,
                              //int rigidbodyIndex,
                              // NativeArray<BurstRigidbody> rigidbodies,

                              NativeArray<float4> positions,
                              //NativeArray<quaternion> orientations,
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
                                 //NativeArray<quaternion> orientations,
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
                //�Ѿ���Ҷ�ӽڵ㣬��Ҷ�ӽڵ������������������������������ײ���
                // check for contact against all triangles:
                for (int dataOffset = node.start; dataOffset < node.start + node.count; ++dataOffset)
                {
                    Triangle t = triangles[header.firstTriangle + dataOffset];
                    float4 v1 = new float4(vertices[header.firstVertex + t.i1], 0);
                    float4 v2 = new float4(vertices[header.firstVertex + t.i2], 0);
                    float4 v3 = new float4(vertices[header.firstVertex + t.i3], 0);
                    BurstAabb triangleBounds = new BurstAabb(v1, v2, v3, shape.contactOffset + collisionMargin);

                    //���ж�aabb�Ƿ��ཻ�����ж϶��㼶��
                    if (triangleBounds.IntersectsAabb(particleBounds))
                    {
                        tri.Cache(v1, v2, v3);

                        //var colliderPoint = BurstLocalOptimization.Optimize<BurstTriangleMesh>(ref this, positions, orientations, radii, simplices, simplexStart, simplexSize,
                        //                                                   ref simplexBary, out float4 simplexPoint, optimizationIterations, optimizationTolerance);
                        var surfacePoint = new SurfacePoint();
                        var convexPoint = positions[particleIndex];
                        var convexThickness = radii[particleIndex];
                        this.Evaluate(convexPoint, convexThickness, quaternion.identity, ref surfacePoint);
                        var nearestPointInTri = surfacePoint;
                        //BurstLocalOptimization.NoOptimize<BurstTriangleMesh>(ref this, positions, radii, simplexIndex);

                        float4 particlePoint = positions[particleIndex];
                        float4 particleVelocity = velocities[particleIndex];
                        float particleRadius = radii[particleIndex].x;


                        float4 rbVelocity = float4.zero;
                        //if (rigidbodyIndex >= 0)
                        //   rbVelocity = BurstMath.GetRigidbodyVelocityAtPoint(rigidbodyIndex, colliderPoint.point, rigidbodies, solverToWorld);
                        //�������ӵ���������������Ծ��������ٶ�
                        float dAB = math.dot(particlePoint - nearestPointInTri.point, nearestPointInTri.normal);
                        float vel = math.dot(particleVelocity - rbVelocity, nearestPointInTri.normal);
                        //�ж�����һ֡���Ƿ����ײ
                        if (vel * dt + dAB <= particleRadius + shape.contactOffset + collisionMargin)
                        {
                            contacts.Enqueue(new BurstContact()
                            {
                                bodyA = particleIndex,
                                bodyB = colliderIndex,
                                pointA = new float4(1, 0, 0, 0),
                                pointB = nearestPointInTri.point,
                                normal = nearestPointInTri.normal,
                                distance = dAB,
                            });
                        }
                    }
                }
            }
        }

    }

}
