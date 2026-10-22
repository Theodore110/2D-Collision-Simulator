using System;

namespace CustomPhysics2D;

/// <summary>
/// Circular collision shape.
/// </summary>
public sealed class Circle : Shape
{
    public float Radius { get; }

    /// <summary>
    /// Local center relative to the owning body's position.
    /// </summary>
    public Vector2 LocalCenter { get; }

    public Circle(
        RigidBody body,
        float radius,
        Vector2 localCenter = default)
        : base(body)
    {
        if (radius <= 0f)
            throw new ArgumentOutOfRangeException(nameof(radius));

        Radius = radius;
        LocalCenter = localCenter;
    }

    public Vector2 Center => Body.Position + LocalCenter;

    /// <summary>
    /// Area of a circle = πr².
    /// </summary>
    public override float ComputeMass(float density)
    {
        return MathF.PI * Radius * Radius * density;
    }

    public override AABB GetAABB()
    {
        Vector2 center = Center;
        Vector2 radius = new(Radius, Radius);

        return new AABB(
            center - radius,
            center + radius);
    }
}