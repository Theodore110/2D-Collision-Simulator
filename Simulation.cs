using System;
using System.Collections.Generic;
using System.Numerics;

namespace CustomPhysics2D;

public enum ShapeKind
{
    Circle,
    Box
}

/// <summary>
/// Everything the renderer and the mouse picking need to know about one body.
/// PhysicsWorld.Bodies only exposes RigidBody, so the shape description is
/// kept next to it here.
/// </summary>
public sealed class Renderable
{
    public Renderable(
        RigidBody body,
        ShapeKind kind,
        float radius,
        Vector2 halfExtents,
        float rotation,
        bool isStatic,
        int colorSeed)
    {
        Body = body;
        Kind = kind;
        Radius = radius;
        HalfExtents = halfExtents;
        Rotation = rotation;
        IsStatic = isStatic;
        ColorSeed = colorSeed;
    }

    public RigidBody Body { get; }
    public ShapeKind Kind { get; }
    public float Radius { get; }
    public Vector2 HalfExtents { get; }
    public float Rotation { get; }
    public bool IsStatic { get; }
    public int ColorSeed { get; }

    /// <summary>
    /// Angle (radians) used for drawing and picking boxes.
    ///
    /// ASSUMPTION: boxes keep the rotation they were created with, because
    /// the demo only ever sets LinearDamping and no angular state. If your
    /// RigidBody/Box exposes a live angle, return it here instead, e.g.
    /// "Body.Angle", and rotating boxes will render correctly.
    /// </summary>
    public float Angle => Rotation;

    public bool Contains(Vector2 point)
    {
        Vector2 d = point - Body.Position;

        if (Kind == ShapeKind.Circle)
        {
           // return d.LengthSquared() <= Radius * Radius;
        }

        float c = MathF.Cos(-Angle);
        float s = MathF.Sin(-Angle);

        float lx = d.X * c - d.Y * s;
        float ly = d.X * s + d.Y * c;

        return MathF.Abs(lx) <= HalfExtents.X
            && MathF.Abs(ly) <= HalfExtents.Y;
    }
}

/// <summary>
/// Owns the PhysicsWorld and the list of drawable bodies, and offers the
/// operations the UI needs: reset, spawn, pick and blast.
/// </summary>
public sealed class Simulation
{
    private readonly Random _rng = new(1234);
    private int _nextColorSeed;

    public Simulation()
    {
        World = CreateWorld();
        Reset(withDemoObjects: true);
    }

    public PhysicsWorld World { get; private set; }

    public List<Renderable> Items { get; } = new();

    public int DynamicCount { get; private set; }

    private static PhysicsWorld CreateWorld()
    {
        return new PhysicsWorld(
            gravity: new Vector2(0f, 9.81f))
        {
            SolverIterations = 8,
            RestitutionThreshold = 0.5f,
            PositionCorrectionPercent = 0.8f,
            PenetrationSlop = 0.005f
        };
    }

    /// <summary>
    /// Builds a fresh world. The engine's API shown in Program.cs has no
    /// RemoveBody, so clearing means replacing the whole world.
    /// </summary>
    public void Reset(bool withDemoObjects)
    {
        World = CreateWorld();
        Items.Clear();
        DynamicCount = 0;

        // Floor plus two walls so nothing leaves the screen. The floor top
        // is at y = 4.5 and its edges are at x = -12 and x = 12.
        AddStaticBox(new Vector2(0f, 5f), new Vector2(12f, 0.5f), 0.15f);
        AddStaticBox(new Vector2(-12.5f, -12.25f), new Vector2(0.5f, 17.75f), 0.15f);
        AddStaticBox(new Vector2(12.5f, -12.25f), new Vector2(0.5f, 17.75f), 0.15f);

        if (!withDemoObjects)
        {
            return;
        }

        for (int i = 0; i < 10; i++)
        {
            float x = -4f + (i % 5) * 2f;
            float y = -8f - (i / 5) * 2f;

            AddCircle(new Vector2(x, y), 0.45f, 0.35f);
        }

        for (int i = 0; i < 10; i++)
        {
            float x = -4f + (i % 5) * 2f;
            float y = -14f - (i / 5) * 2f;

            AddBox(
                new Vector2(x, y),
                new Vector2(0.5f, 0.5f),
                i % 2 == 0 ? 0f : 0.15f,
                0.2f);
        }
    }

    public Renderable Spawn(ShapeKind kind, Vector2 position, float jitter = 0f)
    {
        if (jitter > 0f)
        {
            position += new Vector2(
                Next(-jitter, jitter),
                Next(-jitter, jitter));
        }

        if (kind == ShapeKind.Circle)
        {
            return AddCircle(position, Next(0.28f, 0.65f), 0.35f);
        }

        return AddBox(
            position,
            new Vector2(Next(0.3f, 0.7f), Next(0.3f, 0.7f)),
            Next(-0.4f, 0.4f),
            0.2f);
    }

    /// <summary>Topmost dynamic body under the point, or null.</summary>
    public Renderable? Pick(Vector2 worldPoint)
    {
        for (int i = Items.Count - 1; i >= 0; i--)
        {
            Renderable item = Items[i];

            if (!item.IsStatic && item.Contains(worldPoint))
            {
                return item;
            }
        }

        return null;
    }

    /// <summary>Pushes every dynamic body within radius away from center.</summary>
    public void Blast(Vector2 center, float radius, float strength)
    {
        foreach (Renderable item in Items)
        {
            if (item.IsStatic)
            {
                continue;
            }

            Vector2 d = item.Body.Position - center;
            float dist = d.Length();

            if (dist >= radius)
            {
                continue;
            }

            Vector2 dir = dist > 1e-4f ? d / dist : new Vector2(0f, -1f);
            float falloff = 1f - dist / radius;

            item.Body.Velocity += dir * strength * falloff;
        }
    }

    private Renderable AddCircle(Vector2 position, float radius, float restitution)
    {
        // Mass scales with area; a 0.45 radius circle weighs 1, as in the demo.
        float mass = radius * radius / 0.2025f;

        var body = new RigidBody(
            position,
            mass,
            false,
            restitution);

        body.LinearDamping = 0.02f;

        var circle = new Circle(
            body,
            radius);

        World.AddBody(body, circle);

        var item = new Renderable(
            body,
            ShapeKind.Circle,
            radius,
            Vector2.Zero,
            0f,
            false,
            _nextColorSeed++);

        Items.Add(item);
        DynamicCount++;

        return item;
    }

    private Renderable AddBox(
        Vector2 position,
        Vector2 halfExtents,
        float rotation,
        float restitution)
    {
        // A 0.5 x 0.5 half-extent box weighs 1, as in the demo.
        float mass = halfExtents.X * halfExtents.Y / 0.25f;

        var body = new RigidBody(
            position,
            mass,
            false,
            restitution);

        body.LinearDamping = 0.02f;

        var box = new Box(
            body,
            halfExtents,
            rotation);

        World.AddBody(body, box);

        var item = new Renderable(
            body,
            ShapeKind.Box,
            0f,
            halfExtents,
            rotation,
            false,
            _nextColorSeed++);

        Items.Add(item);
        DynamicCount++;

        return item;
    }

    private void AddStaticBox(Vector2 position, Vector2 halfExtents, float restitution)
    {
        var body = new RigidBody(
            position: position,
            mass: 0f,
            isStatic: true,
            restitution: restitution);

        var box = new Box(
            body,
            halfExtents: halfExtents);

        World.AddBody(body, box);

        Items.Add(new Renderable(
            body,
            ShapeKind.Box,
            0f,
            halfExtents,
            0f,
            true,
            0));
    }

    private float Next(float min, float max)
    {
        return min + (float)_rng.NextDouble() * (max - min);
    }
}