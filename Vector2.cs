using System;
namespace CustomPhysics2D;

/// <summary>
/// Two-dimensional vector used for positions, velocities, forces and directions.
/// </summary>
public readonly struct Vector2 : IEquatable<Vector2>
{
    public readonly float X;
    public readonly float Y;

    public Vector2(float x, float y)
    {
        X = x;
        Y = y;
    }

    /// <summary>Vector with both components equal to zero.</summary>
    public static Vector2 Zero => new(0f, 0f);

    /// <summary>Vector with both components equal to one.</summary>
    public static Vector2 One => new(1f, 1f);

    /// <summary>Unit vector pointing along the positive X axis.</summary>
    public static Vector2 UnitX => new(1f, 0f);

    /// <summary>Unit vector pointing along the positive Y axis.</summary>
    public static Vector2 UnitY => new(0f, 1f);

    /// <summary>
    /// Euclidean length (magnitude) of the vector.
    /// </summary>
    public float Length => MathF.Sqrt(LengthSquared);

    /// <summary>
    /// Squared magnitude. Avoids a square root when only relative lengths matter.
    /// </summary>
    public float LengthSquared => X * X + Y * Y;

    /// <summary>
    /// Returns a unit vector pointing in the same direction.
    /// </summary>
    public Vector2 Normalized()
    {
        float lengthSquared = LengthSquared;

        if (lengthSquared < 1e-12f)
            return Zero;

        float inverseLength = 1f / MathF.Sqrt(lengthSquared);
        return this * inverseLength;
    }

    /// <summary>
    /// Dot product. Measures how strongly two vectors point in the same direction.
    /// </summary>
    public float Dot(Vector2 other)
    {
        return X * other.X + Y * other.Y;
    }

    /// <summary>
    /// 2D cross product. Returns the signed scalar magnitude of the
    /// perpendicular vector produced by the 3D cross product.
    /// </summary>
    public float Cross(Vector2 other)
    {
        return X * other.Y - Y * other.X;
    }

    /// <summary>
    /// Distance between this vector interpreted as a point and another point.
    /// </summary>
    public float Distance(Vector2 other)
    {
        return (this - other).Length;
    }

    /// <summary>
    /// Linear interpolation between this vector and another vector.
    /// t=0 returns this vector and t=1 returns the other vector.
    /// </summary>
    public Vector2 Lerp(Vector2 other, float t)
    {
        return this + (other - this) * t;
    }

    /// <summary>
    /// Returns a vector perpendicular to this vector, rotated 90 degrees counter-clockwise.
    /// </summary>
    public Vector2 Perpendicular()
    {
        return new Vector2(-Y, X);
    }

    public static Vector2 operator +(Vector2 a, Vector2 b)
        => new(a.X + b.X, a.Y + b.Y);

    public static Vector2 operator -(Vector2 a, Vector2 b)
        => new(a.X - b.X, a.Y - b.Y);

    public static Vector2 operator -(Vector2 v)
        => new(-v.X, -v.Y);

    public static Vector2 operator *(Vector2 v, float scalar)
        => new(v.X * scalar, v.Y * scalar);

    public static Vector2 operator *(float scalar, Vector2 v)
        => v * scalar;

    public static Vector2 operator /(Vector2 v, float scalar)
        => new(v.X / scalar, v.Y / scalar);

    public static bool operator ==(Vector2 a, Vector2 b)
        => a.X == b.X && a.Y == b.Y;

    public static bool operator !=(Vector2 a, Vector2 b)
        => !(a == b);

    public bool Equals(Vector2 other) => this == other;

    public override bool Equals(object obj)
        => obj is Vector2 other && Equals(other);

    public override int GetHashCode()
        => HashCode.Combine(X, Y);

    public override string ToString()
        => $"({X:0.###}, {Y:0.###})";
}