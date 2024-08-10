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
        /// 在mesh的本地坐标系中计算最近点
        /// </summary>
        /// <param name="point"></param>
        /// <param name="radii"></param>
        /// <param name="orientation"></param>
        /// <param name="projectedPoint"></param>
        public void Evaluate(float4 point, float4 radii, quaternion orientation, ref SurfacePoint projectedPoint)
        {
            //point = colliderToSolver.InverseTransformPointUnscaled(point);

            float4 nearestPoint = BurstMath.NearestPointOnTri(tri, point, out float4 bary);
            float4 normal = math.normalizesafe(point - nearestPoint);

            projectedPoint.point = nearestPoint + normal * shape.contactOffset;// colliderToSolver.TransformPointUnscaled(nearestPoint + normal * shape.contactOffset);
            projectedPoint.normal = normal;// colliderToSolver.TransformDirection(normal);
        }

        public void Contacts(int colliderIndex,
                             //int rigidbodyIndex,
                            // NativeArray<BurstRigidbody> rigidbodies,

                              NativeArray<float4> positions,
                              //NativeArray<quaternion> orientations,
                              NativeArray<float4> velocities,
                              NativeArray<float4> radii,

                              //NativeArray<int> simplices,
                              in BurstAabb simplexBounds,
                              int simplexIndex,
                              //int simplexStart,
                              //int simplexSize,

                              NativeQueue<BurstContact>.ParallelWriter contacts,
                              int optimizationIterations = 14,
                              float optimizationTolerance = 0.001f)
        {

            BIHTraverse(colliderIndex, simplexIndex,
                        positions, velocities, radii, in simplexBounds, 0, contacts, optimizationIterations, optimizationTolerance);
            
        }

        private void BIHTraverse(int colliderIndex,
                                 //int rigidbodyIndex,
                                 int simplexIndex,
                                 //int simplexStart,
                                 //int simplexSize,
                                 //NativeArray<BurstRigidbody> rigidbodies,
                                 NativeArray<float4> positions,
                                 //NativeArray<quaternion> orientations,
                                 NativeArray<float4> velocities,
                                 NativeArray<float4> radii,
                                 //NativeArray<int> simplices,
                                 in BurstAabb simplexBounds,
                                 int nodeIndex,
                                 NativeQueue<BurstContact>.ParallelWriter contacts,
                                 int optimizationIterations,
                                 float optimizationTolerance)
        {
            var node = bihNodes[header.firstNode + nodeIndex];

            if (node.firstChild >= 0)
            { 
                // visit min node:
                if (simplexBounds.min[node.axis] <= node.leftSplitPlane)
                    BIHTraverse(colliderIndex, simplexIndex,
                                positions, velocities, radii, in simplexBounds,
                                node.firstChild, contacts, optimizationIterations, optimizationTolerance);

                // visit max node:
                if (simplexBounds.max[node.axis] >= node.rightSplitPlane)
                    BIHTraverse(colliderIndex, simplexIndex,
                                positions, velocities, radii, in simplexBounds,
                                node.firstChild + 1, contacts, optimizationIterations, optimizationTolerance);
            }
            else
            {
                // check for contact against all triangles:
                for (int dataOffset = node.start; dataOffset < node.start + node.count; ++dataOffset)
                {
                    Triangle t = triangles[header.firstTriangle + dataOffset];
                    float4 v1 = new float4(vertices[header.firstVertex + t.i1], 0);
                    float4 v2 = new float4(vertices[header.firstVertex + t.i2], 0);
                    float4 v3 = new float4(vertices[header.firstVertex + t.i3], 0);
                    BurstAabb triangleBounds = new BurstAabb(v1, v2, v3, shape.contactOffset + collisionMargin);

                    if (triangleBounds.IntersectsAabb(simplexBounds))
                    {
                        //float4 simplexBary = BurstMath.BarycenterForSimplexOfSize(simplexSize);
                        //tri.Cache(v1 * colliderToSolver.scale, v2 * colliderToSolver.scale, v3 * colliderToSolver.scale);
                        tri.Cache(v1, v2, v3);

                        //var colliderPoint = BurstLocalOptimization.Optimize<BurstTriangleMesh>(ref this, positions, orientations, radii, simplices, simplexStart, simplexSize,
                        //                                                   ref simplexBary, out float4 simplexPoint, optimizationIterations, optimizationTolerance);
                        var colliderPoint = BurstLocalOptimization.NoOptimize<BurstTriangleMesh>(ref this, positions, radii, simplexIndex);
                        float4 simplexPoint = positions[simplexIndex];
                        float4 velocity = velocities[simplexIndex];
                        float simplexRadius = radii[simplexIndex].x;
                        //for (int j = 0; j < simplexSize; ++j)
                        //{
                        //    int particleIndex = simplices[simplexStart + j];
                        //    simplexRadius += radii[particleIndex].x * simplexBary[j];
                        //    velocity += velocities[particleIndex] * simplexBary[j];
                        //}

                        float4 rbVelocity = float4.zero;
                        //if (rigidbodyIndex >= 0)
                         //   rbVelocity = BurstMath.GetRigidbodyVelocityAtPoint(rigidbodyIndex, colliderPoint.point, rigidbodies, solverToWorld);

                        float dAB = math.dot(simplexPoint - colliderPoint.point, colliderPoint.normal);
                        float vel = math.dot(velocity     - rbVelocity, colliderPoint.normal); 

                        if (vel * dt + dAB <= simplexRadius + shape.contactOffset + collisionMargin)
                        {
                            contacts.Enqueue(new BurstContact()
                            {
                                bodyA = simplexIndex,
                                bodyB = colliderIndex,
                                pointA = new float4(1,0,0,0),
                                pointB = colliderPoint.point,
                                normal = colliderPoint.normal,
                            });
                        }
                    }
                }
            }
        }

    }

}
