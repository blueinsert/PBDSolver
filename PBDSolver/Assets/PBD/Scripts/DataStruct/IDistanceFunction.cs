using Unity.Mathematics;

namespace bluebean.Physics.PBD.DataStruct
{
    public interface IDistanceFunction
    {
        /// <summary>
        /// 计算最近点，结果保存在projectedPoint
        /// </summary>
        /// <param name="point"></param>
        /// <param name="radii"></param>
        /// <param name="orientation"></param>
        /// <param name="projectedPoint"></param>
        void Evaluate(float4 point, float4 radii, quaternion orientation, ref SurfacePoint projectedPoint);
    }
}