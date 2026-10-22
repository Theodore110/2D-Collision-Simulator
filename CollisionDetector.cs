using System;

namespace CustomPhysics2D;

/// <summary>
/// Stateless narrow-phase collision detection.
/// </summary>
public static class CollisionDetector
{
    private const float Epsilon = 1e-6f;

    public static CollisionManifold Detect(Shape a, Shape b)
    {
        if (a is Circle circleA && b is Circle circleB)
            return CircleVsCircle(circleA, circleB);

        if (a is Circle circle && b is Box box)
            return CircleVsBox(circle, box);

        if (a is Box boxA && b is Circle circleB2)
        {
            CollisionManifold manifold = CircleVsBox(circleB2, boxA);

            if (!manifold.HasCollision)
                return manifold;

            // CircleVsBox produces a normal Box -> Circle.
            // Swap it because our public convention is A -> B.
            manifold.Normal = -manifold.Normal;
            return manifold;
        }

        if (a is Box box1 && b is Box box2)
            return BoxVsBox(box1, box2);

        return CollisionManifold.None;
    }

    private static CollisionManifold CircleVsCircle(
        Circle a,
        Circle b)
    {
        Vector2 delta = b.Center - a.Center;

        float distanceSquared = delta.LengthSquared;
        float radiusSum = a.Radius + b.Radius;

        if (distanceSquared >= radiusSum * radiusSum)
            return CollisionManifold.None;

        float distance = MathF.Sqrt(distanceSquared);

        Vector2 normal;

        if (distance > Epsilon)
        {
            normal = delta / distance;
        }
        else
        {
            // Coincident centers have no naturally defined normal.
            normal = Vector2.UnitX;
            distance = 0f;
        }

        float penetration = radiusSum - distance;

        Vector2 contact =
            a.Center +
            normal * (a.Radius - penetration * 0.5f);

        return new CollisionManifold
        {
            HasCollision = true,
            Normal = normal,
            PenetrationDepth = penetration,
            ContactCount = 1,
            Contact1 = contact
        };
    }

    private static CollisionManifold CircleVsBox(
        Circle circle,
        Box box)
    {
        Vector2 localCircle = box.WorldToLocal(circle.Center);

        Vector2 closest = new(
            Clamp(
                localCircle.X,
                -box.HalfExtents.X,
                box.HalfExtents.X),

            Clamp(
                localCircle.Y,
                -box.HalfExtents.Y,
                box.HalfExtents.Y));

        Vector2 localDelta = localCircle - closest;
        float distanceSquared = localDelta.LengthSquared;

        if (distanceSquared > circle.Radius * circle.Radius)
            return CollisionManifold.None;

        Vector2 normal;
        float distance;

        if (distanceSquared > Epsilon)
        {
            distance = MathF.Sqrt(distanceSquared);

            // Box -> circle direction.
            normal = box.AxisX * localDelta.X +
                     box.AxisY * localDelta.Y;

            normal = normal / distance;
        }
        else
        {
            // Circle center is inside the box.
            // Find the nearest face and use its outward normal.
            float dx = box.HalfExtents.X - MathF.Abs(localCircle.X);
            float dy = box.HalfExtents.Y - MathF.Abs(localCircle.Y);

            if (dx < dy)
            {
                float sign = localCircle.X >= 0f ? 1f : -1f;
                normal = box.AxisX * sign;

                closest = new Vector2(
                    sign * box.HalfExtents.X,
                    localCircle.Y);

                distance = -dx;
            }
            else
            {
                float sign = localCircle.Y >= 0f ? 1f : -1f;
                normal = box.AxisY * sign;

                closest = new Vector2(
                    localCircle.X,
                    sign * box.HalfExtents.Y);

                distance = -dy;
            }
        }

        float penetration;

        if (distance >= 0f)
            penetration = circle.Radius - distance;
        else
            penetration = circle.Radius + -distance;

        if (penetration < 0f)
            return CollisionManifold.None;

        Vector2 worldClosest =
            box.Center +
            box.AxisX * closest.X +
            box.AxisY * closest.Y;

        return new CollisionManifold
        {
            HasCollision = true,
            Normal = normal,
            PenetrationDepth = penetration,
            ContactCount = 1,
            Contact1 = worldClosest
        };
    }

    private static CollisionManifold BoxVsBox(
        Box a,
        Box b)
    {
        Vector2[] axes =
        {
            a.AxisX,
            a.AxisY,
            b.AxisX,
            b.AxisY
        };

        float minimumOverlap = float.PositiveInfinity;
        Vector2 bestAxis = Vector2.Zero;

        Vector2 centerDelta = b.Center - a.Center;

        // SAT:
        //
        // Project both rectangles onto every candidate separating axis.
        //
        // For each axis:
        //
        // projection radius =
        // |axis · localX| * halfWidth +
        // |axis · localY| * halfHeight
        //
        // If:
        //
        // distance between centers > radiusA + radiusB
        //
        // the projections do not overlap, so the shapes cannot collide.
        for (int i = 0; i < axes.Length; i++)
        {
            Vector2 axis = axes[i].Normalized();

            float radiusA = ProjectRadius(a, axis);
            float radiusB = ProjectRadius(b, axis);

            float centerDistance = MathF.Abs(
                centerDelta.Dot(axis));

            float overlap =
                radiusA +
                radiusB -
                centerDistance;

            if (overlap < 0f)
                return CollisionManifold.None;

            if (overlap < minimumOverlap)
            {
                minimumOverlap = overlap;
                bestAxis = axis;
            }
        }

        // Make normal point from A to B.
        if (centerDelta.Dot(bestAxis) < 0f)
            bestAxis = -bestAxis;

        Vector2 contact = FindApproximateContact(a, b, bestAxis);

        return new CollisionManifold
        {
            HasCollision = true,
            Normal = bestAxis,
            PenetrationDepth = minimumOverlap,
            ContactCount = 1,
            Contact1 = contact
        };
    }

    private static float ProjectRadius(Box box, Vector2 axis)
    {
        return
            MathF.Abs(axis.Dot(box.AxisX)) * box.HalfExtents.X +
            MathF.Abs(axis.Dot(box.AxisY)) * box.HalfExtents.Y;
    }

    private static Vector2 FindApproximateContact(
        Box a,
        Box b,
        Vector2 normal)
    {
        Vector2 pointA = SupportPoint(a, normal);
        Vector2 pointB = SupportPoint(b, -normal);

        return (pointA + pointB) * 0.5f;
    }

    private static Vector2 SupportPoint(
        Box box,
        Vector2 direction)
    {
        float x =
            direction.Dot(box.AxisX) >= 0f
                ? box.HalfExtents.X
                : -box.HalfExtents.X;

        float y =
            direction.Dot(box.AxisY) >= 0f
                ? box.HalfExtents.Y
                : -box.HalfExtents.Y;

        return box.LocalToWorld(new Vector2(x, y));
    }

    private static float Clamp(float value, float min, float max)
    {
        if (value < min)
            return min;

        if (value > max)
            return max;

        return value;
    }
}