using bluebean.Physics.PBD.DataStruct;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace bluebean.Physics.PBD
{

    [BurstCompile]
    public struct ShapeMatchingCalculateRestJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> particleIndices;
        [ReadOnly] public NativeArray<int> firstIndex;
        [ReadOnly] public NativeArray<int> numIndices;
        [ReadOnly] public NativeArray<float4x4> deformation;
        [ReadOnly] public NativeArray<float4> restPositions;
        [ReadOnly] public NativeArray<float> invMasses;

        public NativeArray<float4> restComs;
        public NativeArray<float4x4> Aqq;

        public void Execute(int i)
        {
            RecalculateRestData(i,
                                ref particleIndices,
                                ref firstIndex,
                                ref restComs,
                                ref Aqq,
                                ref deformation,
                                ref numIndices,
                                ref invMasses,
                                ref restPositions);
        }

        public static void RecalculateRestData(int i,
                                                 ref NativeArray<int> particleIndices,
                                                 ref NativeArray<int> firstIndex,
                                                 ref NativeArray<float4> restComs,
                                                 ref NativeArray<float4x4> Aqq,
                                                 ref NativeArray<float4x4> deformation,
                                                 ref NativeArray<int> numIndices,
                                                 ref NativeArray<float> invMasses,
                                                 ref NativeArray<float4> restPositions)
        {
            int k = 0;
            float maximumMass = 10000;

            // initialize rest center of mass and shape matrix:
            restComs[i] = float4.zero;
            Aqq[i] = float4x4.zero;

            float4 restCom = float4.zero;
            float4x4 _Aqq = float4x4.zero;

            // calculate rest center of mass
            for (int j = 0; j < numIndices[i]; ++j)
            {
                k = particleIndices[firstIndex[i] + j];
                float mass = maximumMass;
                if (invMasses[k] > 1.0f / maximumMass)
                    mass = 1.0f / invMasses[k];
                restCom += restPositions[k] * mass;
            }

            if (restCom[3] < BurstMath.epsilon)
                return;

            restCom.xyz /= restCom[3];
            restComs[i] = restCom;
            restCom[3] = 0;

            for (int j = 0; j < numIndices[i]; ++j)
            {
                k = particleIndices[firstIndex[i] + j];
                float mass = maximumMass;
                if (invMasses[k] > 1.0f / maximumMass)
                    mass = 1.0f / invMasses[k];
                var ri = restPositions[k] - restCom;
                _Aqq += mass * BurstMath.multrnsp4(ri, ri);

            }

            _Aqq[3][3] = 1; // so that the determinant is never 0 due to all-zeros row/column.

            Aqq[i] = math.inverse(math.mul(deformation[i], math.mul(_Aqq, math.transpose(deformation[i]))));
        }

    }
}