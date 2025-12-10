using bluebean.Physics.PBD.DataStruct;
using bluebean.Physics.PBD.DataStruct.Native;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{
    /// <summary>
    /// 对于每个粒子与环境碰撞体判断碰撞，产生接触数据
    /// </summary>
    [BurstCompile]
    unsafe struct GenerateContactsJob : IJobParallelFor
    {
        //collider grid:
        [ReadOnly] public NativeMultilevelGrid<int> colliderMultiGrid;
        [DeallocateOnJobCompletion]
        [ReadOnly] public NativeArray<int> gridLevels;

        // particle arrays:
        [ReadOnly] public NativeArray<float4> velocities;
        [ReadOnly] public NativeArray<float4> positions;
        [ReadOnly] public NativeArray<float> invMasses;
        [ReadOnly] public NativeArray<float> radii;
        [ReadOnly] public NativeArray<BurstAabb> particleBounds;

        // collider arrays:
        [ReadOnly] public NativeArray<BurstAffineTransform> colliderTransforms;
        [ReadOnly] public NativeArray<BurstColliderShape> colliderShapes;
        //[ReadOnly] public NativeArray<BurstRigidbody> rigidbodies;
        [ReadOnly] public NativeArray<BurstAabb> colliderBounds;

        // triangle mesh data:
        [ReadOnly] public NativeArray<TriangleMeshHeader> triangleMeshHeaders;
        [ReadOnly] public NativeArray<BIHNode>  triangleMesh_bihNodes;
        [ReadOnly] public NativeArray<Triangle>  triangleMesh_triangles;
        [ReadOnly] public NativeArray<float3>  triangleMesh_vertices;

        [ReadOnly] public float deltaTime;

        // output contacts queue:
        [WriteOnly]
        [NativeDisableParallelForRestriction]
        public NativeQueue<BurstContact>.ParallelWriter contactsQueue;

        public void Execute(int i)
        {
            BurstAabb particleBound = particleBounds[i];//world space

            //在多重网格中查找可能与粒子i碰撞的碰撞体，并加入候选集中
            Unity.Collections.NativeList<int> candidates = new Unity.Collections.NativeList<int>(16, Allocator.Temp);

            // max size of the particle bounds in cells:
            int3 maxSize = new int3(10,10,10);
            //遍历当前multiGrid中的所有level
            for (int l = 0; l < gridLevels.Length; ++l)
            {
                float cellSize = NativeMultilevelGrid<int>.CellSizeOfLevel(gridLevels[l]);
                //粒子bound的覆盖范围, 转为该level下的网格坐标
                int3 minCell = GridHash.Quantize(particleBound.min.xyz, cellSize);
                int3 maxCell = GridHash.Quantize(particleBound.max.xyz, cellSize);
                maxCell = minCell + math.min(maxCell - minCell, maxSize);
                //查找该范围内的所有格子中存放的collider id,作为可能碰撞的候选集
                for (int x = minCell[0]; x <= maxCell[0]; ++x)
                {
                    for (int y = minCell[1]; y <= maxCell[1]; ++y)
                    {
                        for (int z = minCell[2]; z <= maxCell[2]; ++z)
                        {
                            if (colliderMultiGrid.TryGetCellIndex(new int4(x, y, z, gridLevels[l]), out int cellIndex))
                            {
                                var colliderCell = colliderMultiGrid.usedCells[cellIndex];
                                candidates.AddRange(colliderCell.ContentsPointer, colliderCell.Length);
                            }
                        }
                    }
                }
            }

            if (candidates.Length > 0)
            {
                // make sure each candidate collider only shows up once in the array:
                NativeArray<int> uniqueCandidates = candidates.AsArray();
                uniqueCandidates.Sort();
                int uniqueCount = uniqueCandidates.Unique();
                //遍历候选集中每个collider，和粒子做具体的碰撞检测
                for (int k = 0; k < uniqueCount; ++k)
                {
                    int colliderId = uniqueCandidates[k];
                    if (colliderId < colliderShapes.Length)
                    {
                        BurstColliderShape shape = colliderShapes[colliderId];
                        BurstAabb colliderBound = colliderBounds[colliderId];//world space
                        //int rb = shape.rigidbodyIndex;

                        // Expand bounds by rigidbody's linear velocity:
                        //if (rb >= 0)
                        //    colliderBoundsWS.Sweep(rigidbodies[rb].velocity * deltaTime);

                        if (particleBound.IntersectsAabb(in colliderBound))
                        {
                            //narrow phase, 更细致级别的碰撞检测
                            BurstAffineTransform colliderToWorldTransform = colliderTransforms[colliderId];
                            GenerateContacts(in shape, in colliderToWorldTransform, colliderId, i, particleBound);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 细致的几何级别的碰撞侦测
        /// </summary>
        /// <param name="colliderShape"></param>
        /// <param name="colliderToWorldTransform"></param>
        /// <param name="colliderIndex"></param>
        /// <param name="particleIndex"></param>
        /// <param name="particleBound"></param>
        private void GenerateContacts(in BurstColliderShape colliderShape,
                                      in BurstAffineTransform colliderToWorldTransform,
                                      int colliderIndex,
                                      //int rigidbodyIndex,
                                      int particleIndex,
                                      in BurstAabb particleBound)
        {
            float4x4 worldToColliderTransform;
            BurstAabb particleBoundCS;//collider local space

            switch (colliderShape.type)
            {
                case ColliderShapeType.TriangleMesh:

                    if (colliderShape.dataIndex < 0) return;

                    worldToColliderTransform = math.inverse(float4x4.TRS(colliderToWorldTransform.translation.xyz, colliderToWorldTransform.rotation, colliderToWorldTransform.scale.xyz));
                    particleBoundCS = particleBound.Transformed(worldToColliderTransform);

                    BurstTriangleMesh triangleMeshShape = new BurstTriangleMesh()
                    {
                        colliderToWorld = colliderToWorldTransform,
                        shape = colliderShape,
                        header = triangleMeshHeaders[colliderShape.dataIndex],
                        bihNodes =  triangleMesh_bihNodes,
                        triangles =  triangleMesh_triangles,
                        vertices =  triangleMesh_vertices,
                        collisionMargin = 0.01f,
                        dt = deltaTime
                    };

                    triangleMeshShape.Contacts(colliderIndex, positions, velocities, radii, in particleBoundCS,
                        particleIndex, contactsQueue);

                    break;
                    //case ColliderShape.ShapeType.Sphere:
                    //    BurstSphere sphereShape = new BurstSphere() { colliderToSolver = colliderToSolver, shape = shape, dt = deltaTime };
                    //    sphereShape.Contacts(colliderIndex, rigidbodyIndex, rigidbodies, positions, orientations, velocities, radii, simplices, in simplexBoundsSS,
                    //                         simplexIndex, simplexStart, simplexSize, contactsQueue, parameters.surfaceCollisionIterations, parameters.surfaceCollisionTolerance);
                    //    break;
                    //case ColliderShape.ShapeType.Box:
                    //    BurstBox boxShape = new BurstBox() { colliderToSolver = colliderToSolver, shape = shape, dt = deltaTime };
                    //    boxShape.Contacts(colliderIndex, rigidbodyIndex, rigidbodies, positions, orientations, velocities, radii, simplices, in simplexBoundsSS,
                    //                      simplexIndex, simplexStart, simplexSize, contactsQueue, parameters.surfaceCollisionIterations, parameters.surfaceCollisionTolerance);
                    //    break;
                    //case ColliderShape.ShapeType.Capsule:
                    //    BurstCapsule capsuleShape = new BurstCapsule() { colliderToSolver = colliderToSolver, shape = shape, dt = deltaTime };
                    //    capsuleShape.Contacts(colliderIndex, rigidbodyIndex, rigidbodies, positions, orientations, velocities, radii, simplices, in simplexBoundsSS,
                    //                          simplexIndex, simplexStart, simplexSize, contactsQueue, parameters.surfaceCollisionIterations, parameters.surfaceCollisionTolerance);
                    //    break;
                    //case ColliderShape.ShapeType.SignedDistanceField:

                    //    if (shape.dataIndex < 0) return;

                    //    BurstDistanceField distanceFieldShape = new BurstDistanceField()
                    //    {
                    //        colliderToSolver = colliderToSolver,
                    //        solverToWorld = solverToWorld,
                    //        shape = shape,
                    //        distanceFieldHeaders = distanceFieldHeaders,
                    //        dfNodes = distanceFieldNodes,
                    //        dt = deltaTime,
                    //        collisionMargin = parameters.collisionMargin
                    //    };

                    //    distanceFieldShape.Contacts(colliderIndex, rigidbodyIndex, rigidbodies, positions, orientations, velocities, radii, simplices, in simplexBoundsSS,
                    //                                simplexIndex, simplexStart, simplexSize, contactsQueue, parameters.surfaceCollisionIterations, parameters.surfaceCollisionTolerance);

                    //    break;
                    
                    
            }
        }

    }
}
