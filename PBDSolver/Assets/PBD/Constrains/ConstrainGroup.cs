using System.Collections;
using System.Collections.Generic;
using Unity.Jobs;
using UnityEngine;

namespace bluebean.UGFramework.Physics
{
    public abstract class ConstrainGroup
    {

        protected ConstrainType m_constrainType;
        protected ISolver m_solver;

        public int m_constrainCount = 0;

        public ConstrainGroup(ConstrainType type,ISolver solver)
        {
            m_constrainType = type;
            m_solver = solver;
        }

        public abstract JobHandle Solve(JobHandle inputDeps, float substepTime);
        public abstract JobHandle Apply(JobHandle inputDeps, float substepTime);
    }
}
