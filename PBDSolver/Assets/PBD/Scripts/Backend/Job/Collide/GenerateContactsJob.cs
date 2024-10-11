using bluebean.Physics.PBD.DataStruct.Native;
using bluebean.Physics.PBD.DataStruct;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.Physics.PBD
{
    /// <summary>
    /// 对于每个粒子与环境碰撞体判断碰撞，产生接触数据
    /// </summary>
    [BurstCompile]
    unsafe struct GenerateContactsJob : IJobParallelFor
    {
        //collider grid:
        [ReadOnly] public NativeMultilevelGrid<int> colliderGrid;

        [DeallocateOnJobCompletion]
        [ReadOnly] public NativeArray<int> gridLevels;

        // particle arrays:
        [ReadOnly] public NativeArray<float4> velocities;
        [ReadOnly] public NativeArray<float4> positions;
        //[ReadOnly] public NativeArray<quaternion> orientations;
        [ReadOnly] public NativeArray<float> invMasses;
        [ReadOnly] public NativeArray<float4> radii;
        //[ReadOnly] public NativeArray<int> filters;

        // simplex arrays:
        //[ReadOnly] public NativeArray<int> simplices;
        //[ReadOnly] public SimplexCounts simplexCounts;
        [ReadOnly] public NativeArray<BurstAabb> simplexBounds;

        // collider arrays:
        [ReadOnly] public NativeArray<BurstAffineTransform> transforms;
        [ReadOnly] public NativeArray<BurstColliderShape> shapes;
        //[ReadOnly] public NativeArray<BurstCollisionMaterial> collisionMaterials;
        //[ReadOnly] public NativeArray<BurstRigidbody> rigidbodies;
        [ReadOnly] public NativeArray<BurstAabb> bounds;

        // distance field data:
        //[ReadOnly] public NativeArray<DistanceFieldHeader> distanceFieldHeaders;
        //[ReadOnly] public NativeArray<BurstDFNode> distanceFieldNodes;

        // triangle mesh data:
        [ReadOnly] public NativeArray<TriangleMeshHeader> triangleMeshHeaders;
        [ReadOnly] public NativeArray<BIHNode> bihNodes;
        [ReadOnly] public NativeArray<Triangle> triangles;
        [ReadOnly] public NativeArray<float3> vertices;

        // edge mesh data:
        //[ReadOnly] public NativeArray<EdgeMeshHeader> edgeMeshHeaders;
        //[ReadOnly] public NativeArray<BIHNode> edgeBihNodes;
        //[ReadOnly] public NativeArray<Edge> edges;
        //[ReadOnly] public NativeArray<float2> edgeVertices;

        // height field data:
        //[ReadOnly] public NativeArray<HeightFieldHeader> heightFieldHeaders;
        //[ReadOnly] public NativeArray<float> heightFieldSamples;

        // auxiliar data:
        //[ReadOnly] public BurstAffineTransform solverToWorld;
        //[ReadOnly] public BurstAffineTransform worldToSolver;
        [ReadOnly] public float deltaTime;
        //[ReadOnly] public Oni.SolverParameters parameters;

        // output contacts queue:
        [WriteOnly]
        [NativeDisableParallelForRestriction]
        public NativeQueue<BurstContact>.ParallelWriter contactsQueue;

        public void Execute(int i)
        {
            //int simplexStart = simplexCounts.GetSimplexStartAndSize(i, out int simplexSize);
            BurstAabb simplexBoundsWS = simplexBounds[i];

            // get all colliders overlapped by the cell bounds, in all grid levels:
            //BurstAabb simplexBoundsWS = simplexBoundsSS.Transformed(solverToWorld);
            Unity.Collections.NativeList<int> candidates = new Unity.Collections.NativeList<int>(16, Allocator.Temp);

            // max size of the particle bounds in cells:
            int3 maxSize = new int3(10);
            //遍历当前multiGrid中的所有level
            for (int l = 0; l < gridLevels.Length; ++l)
            {
                float cellSize = NativeMultilevelGrid<int>.CellSizeOfLevel(gridLevels[l]);
                //粒子bound的覆盖范围
                int3 minCell = GridHash.Quantize(simplexBoundsWS.min.xyz, cellSize);
                int3 maxCell = GridHash.Quantize(simplexBoundsWS.max.xyz, cellSize);
                maxCell = minCell + math.min(maxCell - minCell, maxSize);
                //查找该范围内的collider,作为可能碰撞的候选集
                for (int x = minCell[0]; x <= maxCell[0]; ++x)
                {
                    for (int y = minCell[1]; y <= maxCell[1]; ++y)
                    {
                        for (int z = minCell[2]; z <= maxCell[2]; ++z)
                        {
                            if (colliderGrid.TryGetCellIndex(new int4(x, y, z, gridLevels[l]), out int cellIndex))
                            {
                                var colliderCell = colliderGrid.usedCells[cellIndex];
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
                // iterate over candidate colliders, generating contacts for each one
                for (int k = 0; k < uniqueCount; ++k)
                {
                    int c = uniqueCandidates[k];
                    if (c < shapes.Length)
                    {
                        BurstColliderShape shape = shapes[c];
                        BurstAabb colliderBoundsWS = bounds[c];
                        //int rb = shape.rigidbodyIndex;

                        // Expand bounds by rigidbody's linear velocity:
                        //if (rb >= 0)
                        //    colliderBoundsWS.Sweep(rigidbodies[rb].velocity * deltaTime);

                        // Expand bounds by collision material's stick distance:
                        //if (shape.materialIndex >= 0)
                        //    colliderBoundsWS.Expand(collisionMaterials[shape.materialIndex].stickDistance);

                        // check if any simplex particle and the collider should collide:
                        bool shouldCollide = true;
                        //var colliderCategory = shape.filter & ObiUtils.FilterCategoryBitmask;
                        //var colliderMask = (shape.filter & ObiUtils.FilterMaskBitmask) >> 16;
                        //for (int j = 0; j < simplexSize; ++j)
                        //{
                        //    var simplexCategory = filters[simplices[simplexStart + j]] & ObiUtils.FilterCategoryBitmask;
                        //    var simplexMask = (filters[simplices[simplexStart + j]] & ObiUtils.FilterMaskBitmask) >> 16;
                        //    shouldCollide |= (simplexCategory & colliderMask) != 0 && (simplexMask & colliderCategory) != 0;
                        //}

                        if (shouldCollide && simplexBoundsWS.IntersectsAabb(in colliderBoundsWS))
                        {
                            // generate contacts for the collider:
                            //BurstAffineTransform colliderToSolver = worldToSolver * transforms[c];
                            BurstAffineTransform colliderToWorld = transforms[c];
                            GenerateContacts(in shape, in colliderToWorld, c,i, simplexBoundsWS);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 具体的几何级别的碰撞侦测
        /// </summary>
        /// <param name="shape"></param>
        /// <param name="colliderToWorld"></param>
        /// <param name="colliderIndex"></param>
        /// <param name="simplexIndex"></param>
        /// <param name="simplexBoundsWS"></param>
        private void GenerateContacts(in BurstColliderShape shape,
                                      in BurstAffineTransform colliderToWorld,
                                      int colliderIndex,
                                      //int rigidbodyIndex,
                                      int simplexIndex,
                                      //int simplexStart,
                                      //int simplexSize,
                                      in BurstAabb simplexBoundsWS)
        {
            //float4x4 solverToCollider;
            float4x4 worldToCollider;
            BurstAabb simplexBoundsCS;

            switch (shape.type)
            {
                case ColliderShapeType.TriangleMesh:

                    if (shape.dataIndex < 0) return;

                    // invert a full matrix here to accurately represent collider bounds scale.
                    //solverToCollider = math.inverse(float4x4.TRS(colliderToSolver.translation.xyz, colliderToSolver.rotation, colliderToSolver.scale.xyz));
                    worldToCollider = math.inverse(float4x4.TRS(colliderToWorld.translation.xyz, colliderToWorld.rotation, colliderToWorld.scale.xyz));
                    simplexBoundsCS = simplexBoundsWS.Transformed(worldToCollider);

                    BurstTriangleMesh triangleMeshShape = new BurstTriangleMesh()
                    {
                        //colliderToSolver = colliderToSolver,
                        //solverToWorld = solverToWorld,
                        colliderToWorld = colliderToWorld,
                        shape = shape,
                        header = triangleMeshHeaders[shape.dataIndex],
                        bihNodes = bihNodes,
                        triangles = triangles,
                        vertices = vertices,
                        collisionMargin = 0.01f,
                        dt = deltaTime
                    };

                    triangleMeshShape.Contacts(colliderIndex, positions, velocities, radii, in simplexBoundsCS,
                        simplexIndex, contactsQueue);

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
                    //case ColliderShape.ShapeType.Heightmap:

                    //    if (shape.dataIndex < 0) return;

                    //    // invert a full matrix here to accurately represent collider bounds scale.
                    //    solverToCollider = math.inverse(float4x4.TRS(colliderToSolver.translation.xyz, colliderToSolver.rotation, colliderToSolver.scale.xyz));
                    //    simplexBoundsCS = simplexBoundsSS.Transformed(solverToCollider);

                    //    BurstHeightField heightmapShape = new BurstHeightField()
                    //    {
                    //        colliderToSolver = colliderToSolver,
                    //        solverToWorld = solverToWorld,
                    //        shape = shape,
                    //        header = heightFieldHeaders[shape.dataIndex],
                    //        heightFieldSamples = heightFieldSamples,
                    //        collisionMargin = parameters.collisionMargin,
                    //        dt = deltaTime
                    //    };

                    //    heightmapShape.Contacts(colliderIndex, rigidbodyIndex, rigidbodies, positions, orientations, velocities, radii, simplices, in simplexBoundsCS,
                    //                                simplexIndex, simplexStart, simplexSize, contactsQueue, parameters.surfaceCollisionIterations, parameters.surfaceCollisionTolerance);

                    //    break;
                    //case ColliderShape.ShapeType.EdgeMesh:

                    //    if (shape.dataIndex < 0) return;

                    //    // invert a full matrix here to accurately represent collider bounds scale.
                    //    solverToCollider = math.inverse(float4x4.TRS(colliderToSolver.translation.xyz, colliderToSolver.rotation, colliderToSolver.scale.xyz));
                    //    simplexBoundsCS = simplexBoundsSS.Transformed(solverToCollider);

                    //    BurstEdgeMesh edgeMeshShape = new BurstEdgeMesh()
                    //    {
                    //        colliderToSolver = colliderToSolver,
                    //        shape = shape,
                    //        header = edgeMeshHeaders[shape.dataIndex],
                    //        edgeBihNodes = edgeBihNodes,
                    //        edges = edges,
                    //        vertices = edgeVertices,
                    //        dt = deltaTime
                    //    };

                    //    edgeMeshShape.Contacts(colliderIndex, rigidbodyIndex, rigidbodies, positions, orientations, velocities, radii, simplices, in simplexBoundsCS,
                    //                                simplexIndex, simplexStart, simplexSize, contactsQueue, parameters.surfaceCollisionIterations, parameters.surfaceCollisionTolerance);

                    //    break;
            }
        }

    }
}
