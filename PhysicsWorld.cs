using System;
using System.Collections.Generic;

namespace CustomPhysics2D;

/// <summary>
/// Owns bodies and performs deterministic fixed-step physics simulation.
/// </summary>
public sealed class PhysicsWorld
{
    private readonly List<RigidBody> _bodies = new();
    private readonly List<Shape> _shapes = new();
    private readonly List<CollisionPair> _contacts = new();

    public Vector2 Gravity { get; set; }

    public int SolverIterations { get; set; } = 8;

    /// <summary>
    /// Impacts slower than this value are treated as non-bouncy.
    /// </summary>
    public float RestitutionThreshold { get; set; } = 0.5f;

    /// <summary>
    /// Percentage of penetration removed per solver correction.
    /// </summary>
    public float PositionCorrectionPercent { get; set; } = 0.8f;

    /// <summary>
    /// Allowed penetration before positional correction begins.
    /// </summary>
    public float PenetrationSlop { get; set; } = 0.01f;

    public IReadOnlyList<RigidBody> Bodies => _bodies;

    public IReadOnlyList<Shape> Shapes => _shapes;

    public PhysicsWorld(
        Vector2 gravity)
    {
        Gravity = gravity;
    }

    public void AddBody(
        RigidBody body,
        Shape shape)
    {
        if (body == null)
            throw new ArgumentNullException(nameof(body));

        if (shape == null)
            throw new ArgumentNullException(nameof(shape));

        if (shape.Body != body)
            throw new InvalidOperationException(
                "The shape must belong to the supplied body.");

        _bodies.Add(body);
        _shapes.Add(shape);
    }

    /// <summary>
    /// Performs exactly one fixed simulation step.
    /// </summary>
    public void Step(float dt)
    {
        if (dt <= 0f)
            throw new ArgumentOutOfRangeException(nameof(dt));

        // ------------------------------------------------------------
        // 1. Apply gravity and integrate velocities.
        //
        // Semi-implicit Euler:
        //
        // a = F / m + gravity
        // v(t+dt) = v(t) + a*dt
        //
        // Semi-implicit Euler then uses the NEW velocity when
        // updating position. It is substantially more stable for
        // mechanical systems than explicit Euler.
        // ------------------------------------------------------------

        for (int i = 0; i < _bodies.Count; i++)
        {
            RigidBody body = _bodies[i];

            if (body.IsStatic)
                continue;

            Vector2 acceleration =
                Gravity +
                body.Force * body.InverseMass;

            body.Acceleration = acceleration;

            body.SetVelocity(
                body.Velocity +
                acceleration * dt);

            if (body.LinearDamping > 0f)
            {
                float damping =
                    MathF.Max(
                        0f,
                        1f - body.LinearDamping * dt);

                body.SetVelocity(
                    body.Velocity * damping);
            }
        }

        // ------------------------------------------------------------
        // 2. Broad phase + narrow phase.
        //
        // For this educational implementation we use an O(n²)
        // pair search. AABB rejection makes the expensive narrow
        // phase run only for spatially close objects.
        // ------------------------------------------------------------

        _contacts.Clear();

        for (int i = 0; i < _shapes.Count; i++)
        {
            Shape shapeA = _shapes[i];

            for (int j = i + 1; j < _shapes.Count; j++)
            {
                Shape shapeB = _shapes[j];

                if (shapeA.Body.IsStatic &&
                    shapeB.Body.IsStatic)
                {
                    continue;
                }

                AABB aabbA = shapeA.GetAABB();
                AABB aabbB = shapeB.GetAABB();

                if (!aabbA.Overlaps(aabbB))
                    continue;

                CollisionManifold manifold =
                    CollisionDetector.Detect(
                        shapeA,
                        shapeB);

                if (!manifold.HasCollision)
                    continue;

                _contacts.Add(
                    new CollisionPair(
                        shapeA.Body,
                        shapeB.Body,
                        manifold));
            }
        }

        // ------------------------------------------------------------
        // 3. Sequential impulse solver.
        //
        // Each iteration improves all contacts. Multiple iterations
        // are important for stacks because a correction applied to
        // one box changes the velocity of its neighbours.
        // ------------------------------------------------------------

        int iterations =
            Math.Max(1, SolverIterations);

        for (int iteration = 0;
             iteration < iterations;
             iteration++)
        {
            for (int i = 0;
                 i < _contacts.Count;
                 i++)
            {
                CollisionPair pair =
                    _contacts[i];

                CollisionResolver.ResolveVelocity(
                    pair.Manifold,
                    pair.A,
                    pair.B,
                    RestitutionThreshold);
            }
        }

        // ------------------------------------------------------------
        // 4. Positional correction.
        // ------------------------------------------------------------

        for (int i = 0;
             i < _contacts.Count;
             i++)
        {
            CollisionPair pair =
                _contacts[i];

            CollisionResolver.PositionalCorrection(
                pair.Manifold,
                pair.A,
                pair.B,
                PositionCorrectionPercent,
                PenetrationSlop);
        }

        // ------------------------------------------------------------
        // 5. Integrate positions.
        //
        // x(t+dt) = x(t) + v(t+dt)*dt
        // ------------------------------------------------------------

        for (int i = 0;
             i < _bodies.Count;
             i++)
        {
            RigidBody body =
                _bodies[i];

            if (body.IsStatic)
                continue;

            body.Position +=
                body.Velocity * dt;

            body.ClearForces();
        }
    }

    private readonly struct CollisionPair
    {
        public readonly RigidBody A;
        public readonly RigidBody B;
        public readonly CollisionManifold Manifold;

        public CollisionPair(
            RigidBody a,
            RigidBody b,
            CollisionManifold manifold)
        {
            A = a;
            B = b;
            Manifold = manifold;
        }
    }
}