namespace CustomPhysics2D;

/// <summary>
/// Axis-aligned bounding box used by the broad phase.
/// </summary>
public readonly struct AABB
{
    public readonly Vector2 Min;
    public readonly Vector2 Max;

    public AABB(Vector2 min, Vector2 max)
    {
        Min = min;
        Max = max;
    }

    /// <summary>
    /// Tests whether two AABBs overlap.
    /// </summary>
    public bool Overlaps(AABB other)
    {
        return Min.X <= other.Max.X &&
               Max.X >= other.Min.X &&
               Min.Y <= other.Max.Y &&
               Max.Y >= other.Min.Y;
    }
}

/// <summary>
/// Base type for geometric collision shapes.
/// </summary>
public abstract class Shape
{
    public RigidBody Body { get; }

    protected Shape(RigidBody body)
    {
        Body = body;
    }

    /// <summary>
    /// Calculates mass from material density and geometric area.
    /// </summary>
    public abstract float ComputeMass(float density);

    /// <summary>
    /// Returns the world-space AABB used for broad-phase collision detection.
    /// </summary>
    public abstract AABB GetAABB();
}