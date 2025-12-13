namespace bluebean.Physics.PBD
{
    public enum ConstrainType
    {
        None = -2,
        Start,

        Collide,
        ParticleCollide,
        Volume,
        Stretch,
        ShapeMatching,

        Max,
    }
}
