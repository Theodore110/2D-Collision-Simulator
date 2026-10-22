namespace CustomPhysics2D;

/// <summary>
/// Physical state of an object.
/// </summary>
public sealed class RigidBody
{
    public Vector2 Position { get; internal set; }
    public Vector2 Velocity { get; internal set; }
    public Vector2 Acceleration { get; internal set; }

    public float Mass { get; private set; }

    /// <summary>
    /// Zero for static/infinite-mass bodies.
    /// Otherwise equal to 1/Mass.
    /// </summary>
    public float InverseMass { get; private set; }

    /// <summary>
    /// Coefficient of restitution. 0 = perfectly inelastic, 1 = perfectly elastic.
    /// </summary>
    public float Restitution
    {
        get => _restitution;
        set => _restitution = Math.Clamp(value, 0f, 1f);
    }

    private float _restitution;

    public bool IsStatic { get; }

    /// <summary>
    /// Accumulated external force waiting to be integrated.
    /// </summary>
    public Vector2 Force { get; private set; }

    /// <summary>
    /// Optional linear damping. 0 means no damping.
    /// </summary>
    public float LinearDamping { get; set; }

    public RigidBody(
        Vector2 position,
        float mass,
        bool isStatic = false,
        float restitution = 0.2f)
    {
        Position = position;
        Velocity = Vector2.Zero;
        Acceleration = Vector2.Zero;
        Force = Vector2.Zero;

        IsStatic = isStatic;

        if (isStatic || mass <= 0f)
        {
            Mass = float.PositiveInfinity;
            InverseMass = 0f;
        }
        else
        {
            Mass = mass;
            InverseMass = 1f / mass;
        }

        Restitution = restitution;
        LinearDamping = 0f;
    }

    /// <summary>
    /// Adds a force. The force is accumulated until the next integration step.
    /// </summary>
    public void ApplyForce(Vector2 force)
    {
        if (IsStatic)
            return;

        Force += force;
    }

    /// <summary>
    /// Instantly changes velocity according to impulse / mass.
    /// </summary>
    public void ApplyImpulse(Vector2 impulse)
    {
        if (IsStatic)
            return;

        Velocity += impulse * InverseMass;
    }

    internal void ClearForces()
    {
        Force = Vector2.Zero;
    }

    internal void SetVelocity(Vector2 velocity)
    {
        if (!IsStatic)
            Velocity = velocity;
    }
}