namespace bluebean.Physics.PBD.DataStruct
{
    public struct TriangleMeshHeader //we need to use the header in the backend, so it must be a struct.
    {
        public int firstNode;
        public int nodeCount;
        public int firstTriangle;
        public int triangleCount;
        public int firstVertex;
        public int vertexCount;

        public TriangleMeshHeader(int firstNode, int nodeCount, int firstTriangle, int triangleCount, int firstVertex, int vertexCount)
        {
            this.firstNode = firstNode;
            this.nodeCount = nodeCount;
            this.firstTriangle = firstTriangle;
            this.triangleCount = triangleCount;
            this.firstVertex = firstVertex;
            this.vertexCount = vertexCount;
        }
    }
}
