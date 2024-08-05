namespace bluebean.UGFramework.DataStruct
{
    public struct CellSpan
    {
        public VectorInt4 min;
        public VectorInt4 max;

        public CellSpan(VectorInt4 min, VectorInt4 max)
        {
            this.min = min;
            this.max = max;
        }
    }
}