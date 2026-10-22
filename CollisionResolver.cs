using System;

namespace CustomPhysics2D;

/// <summary>
/// Resolves detected contacts using sequential impulses.
/// </summary>
public static class CollisionResolver
{
    public static void ResolveVelocity(
        in CollisionManifold manifold,
        RigidBody a,
        RigidBody b,
        float restitutionThreshold)
    {
        if (!manifold.HasCollision)
            return;

        float inverseMassSum =
            a.InverseMass +
            b.InverseMass;

        if (inverseMassSum <= 0f)
            return;

        Vector2 relativeVelocity =
            b.Velocity -
            a.Velocity;

        float velocityAlongNormal =
            relativeVelocity.Dot(manifold.Normal);

        // Positive means the bodies are separating.
        if (velocityAlongNormal > 0f)
            return;

        float restitution =
            MathF.Min(
                a.Restitution,
                b.Restitution);

        // Small impacts are treated as resting contacts.
        // This prevents tiny numerical velocities from creating jitter.
        if (MathF.Abs(velocityAlongNormal) <
            restitutionThreshold)
        {
            restitution = 0f;
        }

        float impulseMagnitude =
            -(1f + restitution) *
            velocityAlongNormal /
            inverseMassSum;

        Vector2 impulse =
            manifold.Normal *
            impulseMagnitude;

        a.ApplyImpulse(-impulse);
        b.ApplyImpulse(impulse);
    }

    /// <summary>
    /// Pushes overlapping bodies apart.
    ///
    /// The correction is intentionally small:
    ///
    /// correction =
    /// max(penetration - slop, 0)
    /// × percentage
    /// × normal
    ///
    /// Slop prevents tiny floating-point overlaps from producing
    /// constant positional oscillation.
    /// </summary>
    public static void PositionalCorrection(
        in CollisionManifold manifold,
        RigidBody a,
        RigidBody b,
        float correctionPercent,
        float penetrationSlop)
    {
        if (!manifold.HasCollision)
            return;

        float inverseMassSum =
            a.InverseMass +
            b.InverseMass;

        if (inverseMassSum <= 0f)
            return;

        float depth =
            MathF.Max(
                manifold.PenetrationDepth -
                penetrationSlop,
                0f);

        if (depth <= 0f)
            return;

        Vector2 correction =
            manifold.Normal *
            (depth *
             correctionPercent /
             inverseMassSum);

        if (!a.IsStatic)
        {
            a.Position -=
                correction * a.InverseMass;
        }

        if (!b.IsStatic)
        {
            b.Position +=
                correction * b.InverseMass;
        }
    }
}