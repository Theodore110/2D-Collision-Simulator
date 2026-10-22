using System;

namespace CustomPhysics2D;

/// <summary>
/// Oriented rectangular collision shape.
/// HalfExtents contains half the width and half the height.
/// </summary>
public sealed class Box : Shape
{
    public Vector2 HalfExtents { get; }

    /// <summary>
    /// Rotation in radians.
    /// </summary>
    public float Rotation { get; }

    public Box(
        RigidBody body,
        Vector2 halfExtents,
        float rotation = 0f)
        : base(body)
    {
        if (halfExtents.X <= 0f || halfExtents.Y <= 0f)
            throw new ArgumentOutOfRangeException(nameof(halfExtents));

        HalfExtents = halfExtents;
        Rotation = rotation;
    }

    public Vector2 Center => Body.Position;

    /// <summary>
    /// Local X axis transformed into world space.
    /// </summary>
    public Vector2 AxisX
    {
        get
        {
            float c = MathF.Cos(Rotation);
            float s = MathF.Sin(Rotation);

            return new Vector2(c, s);
        }
    }

    /// <summary>
    /// Local Y axis transformed into world space.
    /// </summary>
    public Vector2 AxisY
    {
        get
        {
            float c = MathF.Cos(Rotation);
            float s = MathF.Sin(Rotation);

            return new Vector2(-s, c);
        }
    }

    /// <summary>
    /// Rectangle area = width × height = 4hxhy.
    /// </summary>
    public override float ComputeMass(float density)
    {
        float width = HalfExtents.X * 2f;
        float height = HalfExtents.Y * 2f;

        return width * height * density;
    }

    /// <summary>
    /// Converts a local-space point into world space.
    /// </summary>
    public Vector2 LocalToWorld(Vector2 local)
    {
        return Center +
               AxisX * local.X +
               AxisY * local.Y;
    }

    /// <summary>
    /// Converts a world-space point into the box's local coordinates.
    /// </summary>
    public Vector2 WorldToLocal(Vector2 world)
    {
        Vector2 d = world - Center;

        return new Vector2(
            d.Dot(AxisX),
            d.Dot(AxisY));
    }

    public override AABB GetAABB()
    {
        Vector2 x = AxisX * HalfExtents.X;
        Vector2 y = AxisY * HalfExtents.Y;

        Vector2 extent = new(
            MathF.Abs(x.X) + MathF.Abs(y.X),
            MathF.Abs(x.Y) + MathF.Abs(y.Y));

        return new AABB(
            Center - extent,
            Center + extent);
    }
}