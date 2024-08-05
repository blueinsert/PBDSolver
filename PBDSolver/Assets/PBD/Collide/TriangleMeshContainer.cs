using bluebean.UGFramework.DataStruct;
using bluebean.UGFramework.DataStruct.Native;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace bluebean.UGFramework.Physics
{

    public class TriangleMeshContainer
    {
        public Dictionary<Mesh,TriangleMeshHandle> handles;  /**< dictionary indexed by mesh, so that we don't generate data for the same mesh multiple times.*/

        public NativeTriangleMeshHeaderList headers; /**< One header per mesh.*/
        public NativeBIHNodeList bihNodes;
        public NativeTriangleList triangles;
        public NativeVector3List vertices;

        public TriangleMeshContainer()
        {
            handles = new Dictionary<Mesh, TriangleMeshHandle>();
            headers = new NativeTriangleMeshHeaderList();
            bihNodes = new NativeBIHNodeList();
            triangles = new NativeTriangleList();
            vertices = new NativeVector3List();
        }

        public TriangleMeshHandle GetOrCreateTriangleMesh(Mesh source)
        {
            TriangleMeshHandle handle = new TriangleMeshHandle(null);

            if (source != null && !handles.TryGetValue(source, out handle))
            { 
                var sourceTris = source.triangles;
                var sourceVertices = source.vertices;

                // Build a bounding interval hierarchy from the triangles:
                IBounded[] t = new IBounded[sourceTris.Length/3];
                for (int i = 0; i < t.Length; ++i)
                {
                    int t1 = sourceTris[i * 3];
                    int t2 = sourceTris[i * 3 + 1];
                    int t3 = sourceTris[i * 3 + 2];
                    t[i] = new Triangle(t1,t2,t3, sourceVertices[t1], sourceVertices[t2], sourceVertices[t3]);
                }
                var sourceBih = BIH.Build(ref t);

                Triangle[] tris = Array.ConvertAll(t, x => (Triangle)x);

                handle = new TriangleMeshHandle(source, headers.count);
                handles.Add(source, handle);
                headers.Add(new TriangleMeshHeader(bihNodes.count, sourceBih.Length, triangles.count, tris.Length, vertices.count, sourceVertices.Length));

                bihNodes.AddRange(sourceBih);
                triangles.AddRange(tris);
                vertices.AddRange(sourceVertices);
            }

            return handle;
        }

        public void DestroyTriangleMesh(TriangleMeshHandle handle)
        {
            if (handle != null && handle.isValid && handle.index < handles.Count)
            {
                var header = headers[handle.index];

                // Update headers:
                for (int i = 0; i < headers.count; ++i)
                {
                    var h = headers[i];
                    if (h.firstTriangle > header.firstTriangle)
                    {
                        h.firstNode -= header.nodeCount;
                        h.firstTriangle -= header.triangleCount;
                        h.firstVertex -= header.vertexCount;
                        headers[i] = h;
                    }
                }

                // update handles:
                foreach (var pair in handles)
                {
                    if (pair.Value.index > handle.index)
                        pair.Value.index --;
                }

                // Remove nodes, triangles and vertices
                bihNodes.RemoveRange(header.firstNode, header.nodeCount);
                triangles.RemoveRange(header.firstTriangle, header.triangleCount);
                vertices.RemoveRange(header.firstVertex, header.vertexCount);

                // remove header:
                headers.RemoveAt(handle.index);

                // remove the mesh from the dictionary:
                handles.Remove(handle.owner);

                // Invalidate our handle:
                handle.Invalidate();
            }
        }

        public void Dispose()
        {
            if (headers != null)
                headers.Dispose();
            if (triangles != null)
                triangles.Dispose();
            if (vertices != null)
                vertices.Dispose();
            if (bihNodes != null)
                bihNodes.Dispose();
        }

    }
}
