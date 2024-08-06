using System;

namespace bluebean.Physics.PBD.DataStruct
{
    public struct BIHNode
    {
        public int firstChild;     /**< index of the first child node. The second one is right after the first.*/
        public int start;          /**< index of the first element in this node.*/
        public int count;          /**< amount of elements in this node.*/

        public int axis;           /**< axis of the split plane (0,1,2 = x,y,z)*/
        //这里取名min、max有非常大的歧义,min,max不是表示本节点包含的范围，
        //min是左区间子元素的最右范围
        //max是右区间子元素的最左范围,故进行改名
        //public float min;          /**< minimum split plane*/
        //public float max;          /**< maximum split plane*/
        public float leftSplitPlane;          /**< minimum split plane*/
        public float rightSplitPlane;          /**< maximum split plane*/
        public Aabb m_aabb;

        public BIHNode(int start, int count)
        {
            firstChild = -1;
            this.start = start;
            this.count = count;
            axis = 0;
            leftSplitPlane = float.MinValue;
            rightSplitPlane = float.MaxValue;
            m_aabb = new Aabb();
        }
    }
}
