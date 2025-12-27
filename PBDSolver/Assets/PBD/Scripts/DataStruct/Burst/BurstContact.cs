using System.Diagnostics;
using System.Text;
using Unity.Mathematics;

namespace bluebean.Physics.PBD.DataStruct
{
    public struct BurstContact : System.IComparable<BurstContact>
    {
        public float4 pointA; // point A, expressed as simplex barycentric coords for simplices, as a solver-space position for colliders.
        public float4 pointB; // point B, expressed as simplex barycentric coords for simplices, as a solver-space position for colliders.

        public float4 normal;
        public float4 tangent;
        public float4 bitangent;

        public float distance;

        float normalLambda;
        float tangentLambda;
        float bitangentLambda;
        float stickLambda;

        public int bodyA;
        public int bodyB;

        public float normalInvMassA;
        public float tangentInvMassA;
        public float bitangentInvMassA;

        public float normalInvMassB;
        public float tangentInvMassB;
        public float bitangentInvMassB;

        public double pad0; // padding to ensure correct alignment to 128 bytes.


        public float GetNormalLambda()
        {
            return normalLambda;
        }

        public float GetTangentLambda()
        {
            return tangentLambda;
        }

        public int GetParticleCount() { return 2; }
        public int GetParticle(int index) { return index == 0 ? bodyA : bodyB; }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            sb.Append("{");
            sb.Append("bodyA:").Append(bodyA).Append(",");
            sb.Append("bodyB:").Append(bodyB).Append(",");
            sb.Append("normal:").Append(normal).Append(",");
            sb.Append("tangent:").Append(tangent).Append(",");
            sb.Append("bitangent:").Append(bitangent).Append(",");
            sb.Append("TotalTangentInvMass:").Append(TotalTangentInvMass).Append(",");
            sb.Append("TotalBitangentInvMass:").Append(TotalBitangentInvMass).Append(",");
            sb.Append("normalLambda:").Append(normalLambda).Append(",");
            sb.Append("tangentLambda:").Append(tangentLambda);//.Append(",");
            sb.Append("}");
            return sb.ToString();
        }

        public int CompareTo(BurstContact other)
        {
            int first = bodyA.CompareTo(other.bodyA);
            if (first == 0)
                return bodyB.CompareTo(other.bodyB);
            return first;
        }

        public float TotalNormalInvMass
        {
            get { return normalInvMassA + normalInvMassB; }
        }

        public float TotalTangentInvMass
        {
            get { return tangentInvMassA + tangentInvMassB; }
        }

        public float TotalBitangentInvMass
        {
            get { return bitangentInvMassA + bitangentInvMassB; }
        }

        public void CalculateBasis(float4 relativeVelocity)
        {
            tangent = math.normalizesafe(relativeVelocity - math.dot(relativeVelocity, normal) * normal);
            bitangent = math.normalizesafe(new float4(math.cross(normal.xyz, tangent.xyz), 0));
        }

        public void CalculateContactMassesA(float invMass,
                                            float4 position,
                                            float4 contactPoint)
        {
            // initialize inverse linear masses:
            normalInvMassA = tangentInvMassA = bitangentInvMassA = invMass;
        }

        public void CalculateContactMassesB(float invMass,
                                            float4 position,
                                            float4 contactPoint)
        {
            // initialize inverse linear masses:
            normalInvMassB = tangentInvMassB = bitangentInvMassB = invMass;
        }


        public void CalculateContactMassesB(in BurstRigidbody rigidbody, in BurstAffineTransform solver2World)
        {
            float4 rB = solver2World.TransformPoint(pointB) - rigidbody.com;

            // initialize inverse linear masses:
            normalInvMassB = tangentInvMassB = bitangentInvMassB = rigidbody.inverseMass;
            normalInvMassB += BurstMath.RotationalInvMass(rigidbody.inverseInertiaTensor, rB, normal);
            tangentInvMassB += BurstMath.RotationalInvMass(rigidbody.inverseInertiaTensor, rB, tangent);
            bitangentInvMassB += BurstMath.RotationalInvMass(rigidbody.inverseInertiaTensor, rB, bitangent);
        }

        /// <summary>
        /// 粘附力
        /// </summary>
        /// <param name="posA"></param>
        /// <param name="posB"></param>
        /// <param name="stickDistance"></param>
        /// <param name="stickiness"></param>
        /// <param name="dt"></param>
        /// <returns></returns>
        public float SolveAdhesion(float4 posA, float4 posB, float stickDistance, float stickiness, float dt)
        {

            if (TotalNormalInvMass <= 0 || stickDistance <= 0 || stickiness <= 0 || dt <= 0)
                return 0;

            distance = math.dot(posA - posB, normal);

            // calculate stickiness position correction:
            float constraint = stickiness * (1 - math.max(distance / stickDistance, 0)) * dt;

            // calculate lambda multiplier:
            float dlambda = -constraint / TotalNormalInvMass;

            // accumulate lambda:
            float newStickinessLambda = math.min(stickLambda + dlambda, 0);

            // calculate lambda change and update accumulated lambda:
            float lambdaChange = newStickinessLambda - stickLambda;
            stickLambda = newStickinessLambda;

            return lambdaChange;
        }

        /// <summary>
        /// 返回冲量大小
        /// </summary>
        /// <param name="posA"></param>
        /// <param name="posB"></param>
        /// <param name="maxDepenetrationDelta">stepTime时间内最大释放的穿越距离</param>
        /// <returns></returns>
        public float SolvePenetration(float4 posA, float4 posB, float maxDepenetrationDelta)
        {
            if (TotalNormalInvMass <= 0)
                return 0;
            //穿越时为负值
            //project position delta to normal vector:
            distance = math.dot(posA - posB, normal);
            if (distance >= 0) return 0;

            float maxProjection = math.max(-distance - maxDepenetrationDelta, 0);
            float dlambda = -(distance + maxProjection) / TotalNormalInvMass;

            float lambdaChange = dlambda;
            normalLambda = lambdaChange;

            return lambdaChange;
        }

        //public float SolvePenetration(float4 posA, float4 posB, float maxDepenetrationDelta)
        //{
        //    if (TotalNormalInvMass <= 0)
        //        return 0;
        //    //穿越时为负值
        //    //project position delta to normal vector:
        //    distance = math.dot(posA - posB, normal);
        //    //根据速度限制，这一帧结束时，穿透距离应该处于的值
        //    // calculate max projection distance based on depenetration velocity:
        //    float maxProjection = math.max(-distance - maxDepenetrationDelta, 0);
        //    //计算一帧内改变位置需要的冲量大小
        //    // calculate lambda multiplier:
        //    //TotalNormalInvMass是碰撞时两个物体分配冲量大小时的公共分母，
        //    float dlambda = -(distance + maxProjection) / TotalNormalInvMass;

        //    float newLambda = math.max(normalLambda + dlambda, 0);

        //    // calculate lambda change and update accumulated lambda:
        //    float lambdaChange = newLambda - normalLambda;
        //    normalLambda = newLambda;

        //    normalLambda = lambdaChange;
        //    return lambdaChange;
        //}

        public float2 SolveFriction(float4 relativeVelocity, float staticFriction, float dynamicFriction, float dt)
        {
            float2 lambdaChange = float2.zero;

            if (TotalTangentInvMass <= 0 || TotalBitangentInvMass <= 0 ||
                (dynamicFriction <= 0 && staticFriction <= 0)
                || (normalLambda <= 0 && stickLambda <= 0)
                )
            {
                //UnityEngine.Debug.Log("early return");
                return lambdaChange;
            }

            // calculate delta projection on both friction axis:
            float tangentPosDelta = math.dot(relativeVelocity, tangent);
            float bitangentPosDelta = math.dot(relativeVelocity, bitangent);

            // calculate friction pyramid limit:
            //除以时间获得标准单位的冲量(m/s)
            float dynamicFrictionCone = normalLambda / dt * dynamicFriction;
            float staticFrictionCone = normalLambda / dt * staticFriction;

            // tangent impulse:
            // 1/TotalTangentInvMass 即质量加权归一化系数 (1/(w1+w2))
            float tangentLambdaDelta = -tangentPosDelta / TotalTangentInvMass;
            float newTangentLambda =tangentLambdaDelta;
            //tangentLambdaDelta立即为摩擦力引起的冲量减小量，
            //1.切向累计冲量小于静摩擦时，lambdaChange将移除切向位移，物体切向相对不动
            //2.大于静摩擦时，动摩擦力使得物体切向位移减小,减小量dynamicFrictionCone
            //这里的数学处理方法是由于碰撞contact求解可以跨越多个subStep逐渐将物体推开
            //前面几个substep位移累计量小于静摩擦，然后加上后面的大于静摩擦
            if (math.abs(newTangentLambda) > staticFrictionCone)
                newTangentLambda = math.clamp(newTangentLambda, -dynamicFrictionCone, dynamicFrictionCone);

            lambdaChange[0] = newTangentLambda;

            // bitangent impulse:
            float bitangentLambdaDelta = -bitangentPosDelta / TotalBitangentInvMass;
            float newBitangentLambda = bitangentLambdaDelta;

            if (math.abs(newBitangentLambda) > staticFrictionCone)
                newBitangentLambda = math.clamp(newBitangentLambda, -dynamicFrictionCone, dynamicFrictionCone);

            lambdaChange[1] = newBitangentLambda;

            tangentLambda = newTangentLambda + newBitangentLambda;

            return lambdaChange;
        }
    }
}
