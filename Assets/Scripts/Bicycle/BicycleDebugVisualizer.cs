using UnityEngine;

public class BicycleDebugVisualizer : MonoBehaviour
{
    [Header("References")]
    public BicycleController bicycle;

    [Header("Display Options")]
    public bool showHUD = true;
    public bool showGizmos = true;
    public bool showLeanBar = true;

    // ── Styles ─────────────────────────────────────────────────
    private GUIStyle labelStyle;
    private GUIStyle barBgStyle;
    private GUIStyle barFillStyle;
    private bool stylesReady;

    // ──────────────────────────────────────────────────────────
    private void InitStyles()
    {
        if (stylesReady) return;

        labelStyle = new GUIStyle(GUI.skin.label);
        labelStyle.fontSize = 15;
        labelStyle.normal.textColor = Color.white;

        barBgStyle = new GUIStyle(GUI.skin.box);

        barFillStyle = new GUIStyle(GUI.skin.box);

        stylesReady = true;
    }

    // ──────────────────────────────────────────────────────────
    private void OnGUI()
    {
        if (!showHUD || bicycle == null) return;
        InitStyles();

        float lean = bicycle.Lean * Mathf.Rad2Deg;
        float steer = bicycle.Steer * Mathf.Rad2Deg;
        float kmh = bicycle.SpeedKmh;
        var state = bicycle.State;

        float panelW = 260f;
        float panelH = 220f;
        float px = 10f;
        float py = 10f;

        // Background box
        GUI.Box(new Rect(px, py, panelW, panelH), "");

        float lx = px + 10f;
        float ly = py + 10f;
        float lineH = 22f;

        // State
        Color stateColor = state == BicycleController.BikeState.Riding ? Color.green
                         : state == BicycleController.BikeState.Fallen ? Color.red
                         : Color.yellow;
        labelStyle.normal.textColor = stateColor;
        GUI.Label(new Rect(lx, ly, panelW, lineH),
                  $"State:  {state}", labelStyle);
        ly += lineH;

        labelStyle.normal.textColor = Color.white;

        GUI.Label(new Rect(lx, ly, panelW, lineH),
                  $"Speed:  {kmh:F1} km/h", labelStyle);
        ly += lineH;

        GUI.Label(new Rect(lx, ly, panelW, lineH),
                  $"Lean:   {lean:+00.0;-00.0;  0.0}°", labelStyle);
        ly += lineH;

        GUI.Label(new Rect(lx, ly, panelW, lineH),
                  $"Steer:  {steer:+00.0;-00.0;  0.0}°", labelStyle);
        ly += lineH;

        GUI.Label(new Rect(lx, ly, panelW, lineH),
                  $"Ground: {(bicycle.IsGrounded ? "Yes" : "No")}", labelStyle);
        ly += lineH;

        // ── Lean bar ───────────────────────────────────────────
        if (showLeanBar)
        {
            ly += 4f;
            GUI.Label(new Rect(lx, ly, panelW, lineH), "Lean Bar:", labelStyle);
            ly += lineH;

            float barW = panelW - 20f;
            float barH = 14f;
            float centre = lx + barW * 0.5f;
            float maxLean = 70f; // degrees for full bar

            // Background
            GUI.Box(new Rect(lx, ly, barW, barH), "");

            // Fill
            float fillFrac = Mathf.Abs(lean) / maxLean;
            float fillW = Mathf.Clamp(fillFrac * barW * 0.5f, 0f, barW * 0.5f);
            Color fillColor = Color.Lerp(Color.green, Color.red, fillFrac);
            GUI.color = fillColor;

            if (lean >= 0f)
                GUI.Box(new Rect(centre, ly, fillW, barH), "");
            else
                GUI.Box(new Rect(centre - fillW, ly, fillW, barH), "");

            GUI.color = Color.white;
            // Centre line
            GUI.Box(new Rect(centre - 1f, ly, 2f, barH), "");

            ly += barH + 6f;
        }

        // Controls hint
        labelStyle.fontSize = 12;
        labelStyle.normal.textColor = new Color(0.8f, 0.8f, 0.8f);
        GUI.Label(new Rect(lx, ly, panelW, lineH),
                  "W/S=Drive/Brake  A/D=Steer  R=Reset", labelStyle);
    }

    // ──────────────────────────────────────────────────────────
    private void OnDrawGizmos()
    {
        if (!showGizmos || bicycle == null) return;

        BicycleParameters p = bicycle.parameters;
        if (p == null) return;

        Vector3 rear = bicycle.RearContact;
        Vector3 fwd = bicycle.Forward;
        Vector3 right = Vector3.Cross(Vector3.up, fwd).normalized;
        Vector3 front = rear + fwd * p.wheelbase;

        float rR = p.rearWheelRadius;
        float rF = p.frontWheelRadius;

        // Ground contacts
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(rear, 0.04f);
        Gizmos.color = Color.blue;
        Gizmos.DrawSphere(front, 0.04f);

        // Wheelbase
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(rear, front);

        // Wheels
        Gizmos.color = Color.green;
        DrawWheelGizmo(rear + Vector3.up * rR, rR, fwd);
        Gizmos.color = Color.cyan;
        DrawWheelGizmo(front + Vector3.up * rF, rF, fwd);

        // Forward arrow
        Gizmos.color = Color.white;
        Gizmos.DrawRay(rear + Vector3.up * rR, fwd * 0.6f);

        // Lean indicator
        float leanDeg = bicycle.Lean * Mathf.Rad2Deg;
        Vector3 leanDir = Quaternion.AngleAxis(leanDeg, fwd) * Vector3.up;
        Gizmos.color = Color.magenta;
        Gizmos.DrawRay(rear + Vector3.up * rR, leanDir * rR * 1.5f);
    }

    private void DrawWheelGizmo(Vector3 centre, float radius, Vector3 fwd)
    {
        int segs = 24;
        Vector3 right = Vector3.Cross(Vector3.up, fwd).normalized;
        Vector3 up = Vector3.up;

        Vector3 prev = centre + up * radius;
        for (int i = 1; i <= segs; i++)
        {
            float a = i / (float)segs * Mathf.PI * 2f;
            Vector3 next = centre + (up * Mathf.Cos(a) + right * Mathf.Sin(a)) * radius;
            Gizmos.DrawLine(prev, next);
            prev = next;
        }
    }
}