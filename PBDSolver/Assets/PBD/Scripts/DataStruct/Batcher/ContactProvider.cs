using bluebean.Physics.PBD.DataStruct;
using Unity.Collections;

namespace bluebean.Physics.PBD.DataStruct
{
    public struct ContactProvider
    {
        public NativeArray<BurstContact> contacts;
        public NativeArray<BurstContact> sortedContacts;

        public int GetConstraintCount()
        {
            return contacts.Length;
        }

        public int GetParticleCount(int constraintIndex)
        {
            return 2;
        }

        public int GetParticle(int constraintIndex, int index)
        {
            if(index == 0)
            {
                return contacts[constraintIndex].bodyA;
            }else
            {
                return contacts[constraintIndex].bodyB;
            }  
        }

        public void WriteSortedConstraint(int constraintIndex, int sortedIndex)
        {
            sortedContacts[sortedIndex] = contacts[constraintIndex];
        }
    }
}