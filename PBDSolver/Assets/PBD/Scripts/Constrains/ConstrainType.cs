namespace bluebean.Physics.PBD
{
    public enum ConstrainType
    {
        None = -2,
        Start,

        Collide,
        Friction,
        ParticleCollide,
        ParticleFriction,
        Volume,
        Stretch,
        ShapeMatching,

        Max,
    }
}
