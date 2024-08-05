using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace bluebean.UGFramework.Physics
{
    public static class BurstMath
    {
        /// <summary>
        /// 计算四面体体积
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        /// <param name="p4"></param>
        /// <returns></returns>
        public static float CalcTetVolume(float3 p1, float3 p2, float3 p3, float3 p4)
        {
            var temp = math.cross(p2 - p1, p3 - p1);
            float res = math.dot(p4 - p1, temp);
            res *= (1.0f / 6);
            return res;
        }
    }
}
