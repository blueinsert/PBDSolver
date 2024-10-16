using UnityEngine;

namespace bluebean.Physics.PBD.DataStruct
{
    public struct AffineTransform
    {
        public Vector4 translation;
        public Vector4 scale;
        public Quaternion rotation;

        public AffineTransform(Vector4 translation, Quaternion rotation, Vector4 scale)
        {
            // make sure there are good values in the 4th component:
            translation[3] = 0;
            scale[3] = 1;

            this.translation = translation;
            this.rotation = rotation;
            this.scale = scale;
        }

        public void FromTransform(Transform source, bool is2D = false)
        {
            translation = source.position;
            rotation = source.rotation;
            scale = source.lossyScale;

            if (is2D)
            {
                translation[2] = 0;
            }
        }

        public void FromMatrix(Matrix4x4 matrix, bool is2D = false)
        {
            translation = matrix.GetPosition();
            rotation = matrix.rotation;
            scale = matrix.lossyScale;

            if (is2D)
            {
                translation[2] = 0;
            }
        }

        public Matrix4x4 ToMatrix()
        {
            var local2WorldMatrix = Matrix4x4.TRS(translation,rotation, scale);
            return local2WorldMatrix;
        }

    }
}
