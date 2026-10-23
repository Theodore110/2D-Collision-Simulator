using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Numerics;
using System.Windows.Forms;

namespace CustomPhysics2D;

public static class Program
{
    [STAThread]
    public static void Main()
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new SimulationForm());
    }
}

/// <summary>
/// Window that draws the simulation with GDI+ and turns mouse and keyboard
/// input into spawn, grab, blast and view operations.
/// </summary>
public sealed class SimulationForm : Form
{
    private const float FixedDt = 1f / 60f;

    // The broadphase cost grows quickly with body count, so the brush stops here.
    private const int MaxDynamicBodies = 600;

    private static readonly Color BackgroundColor = Color.FromArgb(24, 26, 33);
    private static readonly SolidBrush StaticBrush = new(Color.FromArgb(88, 94, 108));
    private static readonly SolidBrush TextBrush = new(Color.FromArgb(225, 228, 235));
    private static readonly SolidBrush DimBrush = new(Color.FromArgb(150, 156, 170));
    private static readonly SolidBrush PauseBrush = new(Color.FromArgb(255, 196, 87));
    private static readonly SolidBrush LimitBrush = new(Color.FromArgb(255, 120, 120));
    private static readonly Pen OutlinePen = new(Color.FromArgb(20, 22, 28), 2f) { LineJoin = LineJoin.Round };
    private static readonly Pen StaticPen = new(Color.FromArgb(130, 137, 153), 2f) { LineJoin = LineJoin.Round };
    private static readonly Pen HighlightPen = new(Color.White, 3f) { LineJoin = LineJoin.Round };
    private static readonly Pen GrabPen = new(Color.FromArgb(200, 255, 255, 255), 2f);
    private static readonly Pen GhostPen = new(Color.FromArgb(90, 255, 255, 255), 1.5f);
    private static readonly Font HudFont = new("Consolas", 10f);

    // One brush per hue; the hue is derived from the body's color seed.
    private static readonly Dictionary<int, SolidBrush> BodyBrushes = new();

    private readonly Simulation _sim = new();
    private readonly Camera _camera = new(40f, new Vector2(640f, 480f));
    private readonly System.Windows.Forms.Timer _timer = new() { Interval = 15 };
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private readonly PointF[] _poly = new PointF[4];

    private double _lastTime;
    private float _accumulator;
    private int _step;

    private float _fps;
    private float _fpsTime;
    private int _fpsFrames;

    private bool _paused;
    private bool _stepOnce;
    private bool _showHelp = true;
    private bool _rightDown;
    private bool _panning;
    private ShapeKind _brush = ShapeKind.Circle;

    private Renderable? _grabbed;
    private Vector2 _grabOffset;
    private Vector2 _mouse;
    private float _brushTimer;
    private Size _lastClientSize;

    public SimulationForm()
    {
        Text = "CustomPhysics2D";
        ClientSize = new Size(1280, 720);
        StartPosition = FormStartPosition.CenterScreen;
        BackColor = BackgroundColor;

        SetStyle(
            ControlStyles.UserPaint |
            ControlStyles.AllPaintingInWmPaint |
            ControlStyles.OptimizedDoubleBuffer |
            ControlStyles.ResizeRedraw,
            true);

        // Put the floor (world y = 4.5) near the bottom edge.
        _camera.Offset = new Vector2(ClientSize.Width / 2f, ClientSize.Height * 2f / 3f);
        _lastClientSize = ClientSize;

        _timer.Tick += (_, _) => Tick();
        _timer.Start();
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _timer.Dispose();
        }

        base.Dispose(disposing);
    }

    // ------------------------------------------------------------------
    // Simulation loop
    // ------------------------------------------------------------------

    private void Tick()
    {
        double now = _clock.Elapsed.TotalSeconds;

        // Clamped so a stall (window drag, debugger) cannot trigger a burst
        // of hundreds of physics steps.
        float frameTime = (float)Math.Min(now - _lastTime, 0.25);
        _lastTime = now;

        _fpsFrames++;
        _fpsTime += frameTime;

        if (_fpsTime >= 0.5f)
        {
            _fps = _fpsFrames / _fpsTime;
            _fpsFrames = 0;
            _fpsTime = 0f;
        }

        // Holding right paints a stream of bodies.
        if (_rightDown)
        {
            _brushTimer -= frameTime;

            if (_brushTimer <= 0f && _sim.DynamicCount < MaxDynamicBodies)
            {
                _sim.Spawn(_brush, _camera.ScreenToWorld(_mouse), jitter: 0.25f);
                _brushTimer = 0.05f;
            }
        }

        if (!_paused)
        {
            _accumulator += frameTime;
        }
        else if (_stepOnce)
        {
            _accumulator = FixedDt;
        }

        _stepOnce = false;

        while (_accumulator >= FixedDt)
        {
            if (_grabbed != null)
            {
                DragToward(_grabbed, _camera.ScreenToWorld(_mouse) - _grabOffset);
            }

            _sim.World.Step(FixedDt);

            _accumulator -= FixedDt;
            _step++;
        }

        Invalidate();
    }

    /// <summary>
    /// Spring-like drag: steer the body's velocity toward the target. Letting
    /// go keeps the current velocity, so a fast drag throws the body.
    /// </summary>
    private static void DragToward(Renderable item, Vector2 target)
    {
        Vector2 v = (target - item.Body.Position) * 12f;

        float speed = v.Length;

        if (speed > 25f)
        {
            v *= 25f / speed;
        }

        item.Body.Velocity = v;
    }

    // ------------------------------------------------------------------
    // Input
    // ------------------------------------------------------------------

    protected override void OnMouseDown(MouseEventArgs e)
    {
        base.OnMouseDown(e);

        _mouse = new Vector2(e.X, e.Y);
        Vector2 world = _camera.ScreenToWorld(_mouse);

        if (e.Button == MouseButtons.Left)
        {
            _grabbed = _sim.Pick(world);

            if (_grabbed != null)
            {
                _grabOffset = world - _grabbed.Body.Position;
            }
            else if (_sim.DynamicCount < MaxDynamicBodies)
            {
                _sim.Spawn(_brush, world);
            }
        }
        else if (e.Button == MouseButtons.Right)
        {
            _rightDown = true;
            _brushTimer = 0f;
        }
        else if (e.Button == MouseButtons.Middle)
        {
            _panning = true;
        }
    }

    protected override void OnMouseUp(MouseEventArgs e)
    {
        base.OnMouseUp(e);

        if (e.Button == MouseButtons.Left)
        {
            _grabbed = null;
        }
        else if (e.Button == MouseButtons.Right)
        {
            _rightDown = false;
        }
        else if (e.Button == MouseButtons.Middle)
        {
            _panning = false;
        }
    }

    protected override void OnMouseMove(MouseEventArgs e)
    {
        base.OnMouseMove(e);

        var now = new Vector2(e.X, e.Y);

        if (_panning)
        {
            _camera.Offset += now - _mouse;
        }

        _mouse = now;
    }

    protected override void OnMouseWheel(MouseEventArgs e)
    {
        base.OnMouseWheel(e);

        _camera.ZoomAt(
            new Vector2(e.X, e.Y),
            MathF.Pow(1.1f, e.Delta / 120f));
    }

    protected override void OnKeyDown(KeyEventArgs e)
    {
        base.OnKeyDown(e);

        switch (e.KeyCode)
        {
            case Keys.D1:
                _brush = ShapeKind.Circle;
                break;

            case Keys.D2:
                _brush = ShapeKind.Box;
                break;

            case Keys.Space:
                _paused = !_paused;
                break;

            case Keys.N:
                _stepOnce = _paused;
                break;

            case Keys.H:
                _showHelp = !_showHelp;
                break;

            case Keys.R:
                _sim.Reset(withDemoObjects: true);
                _grabbed = null;
                break;

            case Keys.C:
                _sim.Reset(withDemoObjects: false);
                _grabbed = null;
                break;

            case Keys.F:
                {
                    Point p = PointToClient(Cursor.Position);
                    _sim.Blast(_camera.ScreenToWorld(new Vector2(p.X, p.Y)), 6f, 14f);
                    break;
                }

            case Keys.Escape:
                Close();
                break;

            default:
                return;
        }

        e.Handled = true;
    }

    // Losing focus mid-drag would swallow the button-up, leaving things stuck.
    protected override void OnDeactivate(EventArgs e)
    {
        base.OnDeactivate(e);

        _grabbed = null;
        _rightDown = false;
        _panning = false;
    }

    // Keep the same world point at the center of the window when resized.
    protected override void OnClientSizeChanged(EventArgs e)
    {
        base.OnClientSizeChanged(e);

        _camera.Offset += new Vector2(
            ClientSize.Width - _lastClientSize.Width,
            ClientSize.Height - _lastClientSize.Height) / 2f;

        _lastClientSize = ClientSize;
    }

    // ------------------------------------------------------------------
    // Drawing
    // ------------------------------------------------------------------

    protected override void OnPaint(PaintEventArgs e)
    {
        Graphics g = e.Graphics;

        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.Clear(BackgroundColor);

        foreach (Renderable item in _sim.Items)
        {
            DrawItem(g, item, item == _grabbed);
        }

        Vector2 mouseWorld = _camera.ScreenToWorld(_mouse);

        if (_grabbed != null)
        {
            Vector2 anchor = _camera.WorldToScreen(_grabbed.Body.Position + _grabOffset);

            g.DrawLine(GrabPen, anchor.X, anchor.Y, _mouse.X, _mouse.Y);
            g.DrawEllipse(GrabPen, _mouse.X - 4f, _mouse.Y - 4f, 8f, 8f);
        }
        else if (_sim.Pick(mouseWorld) == null)
        {
            DrawGhost(g);
        }

        DrawHud(g);
    }

    private void DrawItem(Graphics g, Renderable item, bool highlighted)
    {
        Vector2 center = _camera.WorldToScreen(item.Body.Position);

        SolidBrush fill = BrushFor(item);
        Pen outline = item.IsStatic
            ? StaticPen
            : (highlighted ? HighlightPen : OutlinePen);

        if (item.Kind == ShapeKind.Circle)
        {
            float r = item.Radius * _camera.Scale;

            g.FillEllipse(fill, center.X - r, center.Y - r, r * 2f, r * 2f);
            g.DrawEllipse(outline, center.X - r, center.Y - r, r * 2f, r * 2f);

            // A tick from the center so spin would be visible if the engine rotates circles.
            g.DrawLine(outline, center.X, center.Y, center.X + r, center.Y);
            return;
        }

        Vector2 half = item.HalfExtents;
        Vector2 pos = item.Body.Position;

        float c = MathF.Cos(item.Angle);
        float s = MathF.Sin(item.Angle);

        _poly[0] = ToPoint(_camera.WorldToScreen(pos + Rotate(new Vector2(-half.X, -half.Y), c, s)));
        _poly[1] = ToPoint(_camera.WorldToScreen(pos + Rotate(new Vector2(half.X, -half.Y), c, s)));
        _poly[2] = ToPoint(_camera.WorldToScreen(pos + Rotate(new Vector2(half.X, half.Y), c, s)));
        _poly[3] = ToPoint(_camera.WorldToScreen(pos + Rotate(new Vector2(-half.X, half.Y), c, s)));

        g.FillPolygon(fill, _poly);
        g.DrawPolygon(outline, _poly);
    }

    private void DrawGhost(Graphics g)
    {
        if (_brush == ShapeKind.Circle)
        {
            float r = 0.45f * _camera.Scale;

            g.DrawEllipse(GhostPen, _mouse.X - r, _mouse.Y - r, r * 2f, r * 2f);
            return;
        }

        float half = 0.5f * _camera.Scale;

        g.DrawRectangle(GhostPen, _mouse.X - half, _mouse.Y - half, half * 2f, half * 2f);
    }

    private void DrawHud(Graphics g)
    {
        float y = 10f;

        g.DrawString(
            $"FPS {_fps:0}   Step {_step}   Bodies {_sim.Items.Count}   Brush: {_brush}",
            HudFont, TextBrush, 12f, y);
        y += 22f;

        if (_paused)
        {
            g.DrawString("PAUSED  (N = single step)", HudFont, PauseBrush, 12f, y);
            y += 22f;
        }

        if (_sim.DynamicCount >= MaxDynamicBodies)
        {
            g.DrawString("Body limit reached  (C clears)", HudFont, LimitBrush, 12f, y);
            y += 22f;
        }

        if (!_showHelp)
        {
            g.DrawString("H: help", HudFont, DimBrush, 12f, y);
            return;
        }

        string[] lines =
        {
            "Left click body   grab and throw",
            "Left click empty  spawn brush shape",
            "Hold right mouse  paint bodies",
            "1 / 2             circle / box brush",
            "F                 blast away from cursor",
            "Space / N         pause / single step",
            "R / C             reset demo / clear all",
            "Wheel / middle    zoom / pan",
            "H                 hide help   Esc quit"
        };

        foreach (string line in lines)
        {
            g.DrawString(line, HudFont, DimBrush, 12f, y);
            y += 18f;
        }
    }

    private static SolidBrush BrushFor(Renderable item)
    {
        if (item.IsStatic)
        {
            return StaticBrush;
        }

        int hue = (item.ColorSeed * 47) % 360;

        if (!BodyBrushes.TryGetValue(hue, out SolidBrush? brush))
        {
            brush = new SolidBrush(ColorFromHsv(hue, 0.55f, 0.95f));
            BodyBrushes[hue] = brush;
        }

        return brush;
    }

    private static Color ColorFromHsv(float hue, float saturation, float value)
    {
        float chroma = value * saturation;
        float x = chroma * (1f - MathF.Abs((hue / 60f) % 2f - 1f));
        float m = value - chroma;

        (float r, float gr, float b) = hue switch
        {
            < 60f => (chroma, x, 0f),
            < 120f => (x, chroma, 0f),
            < 180f => (0f, chroma, x),
            < 240f => (0f, x, chroma),
            < 300f => (x, 0f, chroma),
            _ => (chroma, 0f, x)
        };

        return Color.FromArgb(
            255,
            (int)((r + m) * 255f),
            (int)((gr + m) * 255f),
            (int)((b + m) * 255f));
    }

    private static Vector2 Rotate(Vector2 v, float c, float s)
    {
        return new Vector2(v.X * c - v.Y * s, v.X * s + v.Y * c);
    }

    private static PointF ToPoint(Vector2 v)
    {
        return new PointF(v.X, v.Y);
    }
}

/// <summary>Maps world meters to screen pixels. World +Y points down.</summary>
public sealed class Camera
{
    private const float MinScale = 8f;
    private const float MaxScale = 200f;

    public Camera(float scale, Vector2 offset)
    {
        Scale = scale;
        Offset = offset;
    }

    /// <summary>Pixels per meter.</summary>
    public float Scale { get; private set; }

    /// <summary>Screen position of the world origin.</summary>
    public Vector2 Offset { get; set; }

    public Vector2 WorldToScreen(Vector2 world)
    {
        return Offset + world * Scale;
    }

    public Vector2 ScreenToWorld(Vector2 screen)
    {
        return (screen - Offset) / Scale;
    }

    public void ZoomAt(Vector2 screenPoint, float factor)
    {
        Vector2 before = ScreenToWorld(screenPoint);

        Scale = Math.Clamp(Scale * factor, MinScale, MaxScale);

        // Keep the world point under the cursor fixed.
        Offset = screenPoint - before * Scale;
    }
}