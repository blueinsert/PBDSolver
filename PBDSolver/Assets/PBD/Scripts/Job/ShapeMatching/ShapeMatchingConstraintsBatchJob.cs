using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using bluebean.Physics.PBD.DataStruct;
using System.Diagnostics;

namespace bluebean.Physics.PBD
{
    [BurstCompile]
    public struct ShapeMatchingConstraintsBatchJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> particleIndices;
        [ReadOnly] public NativeArray<int> firstIndex;
        [ReadOnly] public NativeArray<int> numIndices;
        [ReadOnly] public NativeArray<float> shapeMaterialParameters;
        [ReadOnly] public NativeArray<float4> restComs;
       
        [ReadOnly] public NativeArray<float4x4> Aqq;
        public NativeArray<float4> coms;
        public NativeArray<quaternion> constraintOrientations;
        public NativeArray<float4x4> linearTransforms;
        public NativeArray<float4x4> deformation;

        [ReadOnly] public NativeArray<float4> positions;
        [ReadOnly] public NativeArray<float4> restPositions;
        [ReadOnly] public NativeArray<float> invMasses;

        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<float4> deltas;
        [NativeDisableContainerSafetyRestriction][NativeDisableParallelForRestriction] public NativeArray<int> counts;

        [ReadOnly] public float deltaTime;

        public void Execute(int i)
        {
            int k;
            float maximumMass = 10000;

            coms[i] = float4.zero;
            float4x4 Apq = float4x4.zero;
            float sumMass = 0;
            // calculate center of mass
            for (int j = 0; j < numIndices[i]; ++j)
            {
                k = particleIndices[firstIndex[i] + j];

                float mass = maximumMass;
                if (invMasses[k] > 1.0f / maximumMass)
                    mass = 1.0f / invMasses[k];

                coms[i] += positions[k] * mass;
                sumMass += mass;
            }

            if (sumMass < BurstMath.epsilon)
                return;

            coms[i] /= sumMass;

            float4 restCom = restComs[i];
            restCom[3] = 0;

            // calculate Apq
            for (int j = 0; j < numIndices[i]; ++j)
            {
                k = particleIndices[firstIndex[i] + j];

                float mass = maximumMass;
                if (invMasses[k] > 1.0f / maximumMass)
                    mass = 1.0f / invMasses[k];

                float4 restPosition = restPositions[k];
                restPosition[3] = 0;

                Apq += mass * BurstMath.multrnsp4(positions[k] - coms[i], restPosition - restCom);
            }

            // calculate optimal transform including plastic deformation:
            float4x4 Apq_def = math.mul(Apq, math.transpose(deformation[i]));
            Apq_def[3][3] = 1;

            // reconstruct full best-matching linear transform:
            linearTransforms[i] = math.mul(Apq_def, Aqq[i]);

            // extract rotation from transform matrix, using warmstarting and few iterations:
            constraintOrientations[i] = BurstMath.ExtractRotation(Apq_def, constraintOrientations[i], 2);

            // finally, obtain rotation matrix:
            float4x4 R = constraintOrientations[i].toMatrix();
            R[3][3] = 0;

            // calculate and accumulate particle goal positions:
            float4 goal;
            float4x4 transform = math.mul(R, deformation[i]);
            for (int j = 0; j < numIndices[i]; ++j)
            {
                k = particleIndices[firstIndex[i] + j];
                goal = coms[i] + math.mul(transform, restPositions[k] - restCom);
                deltas[k] += (goal - positions[k]) * shapeMaterialParameters[i * 5];
                counts[k]++;
            }

            // update plastic deformation:
            float plastic_yield = shapeMaterialParameters[i * 5 + 1];
            float plastic_creep = shapeMaterialParameters[i * 5 + 2];
            float plastic_recovery = shapeMaterialParameters[i * 5 + 3];
            float max_deform = shapeMaterialParameters[i * 5 + 4];

            // if we are allowed to absorb deformation:
            if (plastic_creep > 0)
            {
                R[3][3] = 1;

                // get scale matrix (A = RS so S = Rt * A) and its deviation from the identity matrix:
                float4x4 deform_matrix = math.mul(math.transpose(R), linearTransforms[i]) - float4x4.identity;

                // if the amount of deformation exceeds the yield threshold:
                float norm = deform_matrix.frobeniusNorm();
                if (norm > plastic_yield)
                {
                    // deform the shape permanently:
                    deformation[i] = math.mul(float4x4.identity + plastic_creep * deform_matrix, deformation[i]);

                    // clamp deformation so that it does not exceed a percentage;
                    deform_matrix = deformation[i] - float4x4.identity;
                    norm = deform_matrix.frobeniusNorm();
                    if (norm > max_deform)
                    {
                        deformation[i] = float4x4.identity + max_deform * deform_matrix / norm;
                    }

                    // if we cannot recover from plastic deformation, recalculate rest shape now:
                    if (plastic_recovery == 0)
                        ShapeMatchingCalculateRestJob.RecalculateRestData(i,
                                ref particleIndices,
                                ref firstIndex,
                                ref restComs,
                                ref Aqq,
                                ref deformation,
                                ref numIndices,
                                ref invMasses,
                                ref restPositions);
                }
            }

            // if we can recover from plastic deformation, lerp towards non-deformed shape and recalculate rest shape:
            if (plastic_recovery > 0)
            {
                deformation[i] += (float4x4.identity - deformation[i]) * math.min(plastic_recovery * deltaTime, 1.0f);
                ShapeMatchingCalculateRestJob.RecalculateRestData(i,
                                ref particleIndices,
                                ref firstIndex,
                                ref restComs,
                                ref Aqq,
                                ref deformation,
                                ref numIndices,
                                ref invMasses,
                                ref restPositions);
            }
        }
    }
}