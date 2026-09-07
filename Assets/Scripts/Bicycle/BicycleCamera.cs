using UnityEngine;
using UnityEngine.InputSystem;

public class BicycleCamera : MonoBehaviour
{
    [Header("Target")]
    public BicycleController target;

    [Header("Camera Modes")]
    public CameraMode mode = CameraMode.ThirdPerson;
    public enum CameraMode { ThirdPerson, TopDown, FirstPerson }

    [Header("Third Person Settings")]
    public float tpDistance = 4f;
    public float tpHeight = 1.8f;
    public float tpLookAhead = 2f;
    public float tpLeanTilt = 5f;   // degrees of camera tilt matching lean

    [Header("Top Down Settings")]
    public float tdHeight = 12f;

    [Header("First Person Settings")]
    public float fpHeadHeight = 1.1f; // above rear axle

    [Header("Smoothing")]
    public float posSmooth = 6f;
    public float rotSmooth = 8f;

    private Vector3 velRef = Vector3.zero;
    private Quaternion lastRot = Quaternion.identity;

    // ──────────────────────────────────────────────────────────
    private void LateUpdate()
    {
        if (target == null) return;

        switch (mode)
        {
            case CameraMode.ThirdPerson: UpdateThirdPerson(); break;
            case CameraMode.TopDown: UpdateTopDown(); break;
            case CameraMode.FirstPerson: UpdateFirstPerson(); break;
        }
    }

    // ──────────────────────────────────────────────────────────
    private void UpdateThirdPerson()
    {
        Vector3 rearPos = target.RearContact
                        + Vector3.up * target.parameters.rearWheelRadius;
        Vector3 fwd = target.Forward;

        Vector3 desired = rearPos
                        - fwd * tpDistance
                        + Vector3.up * tpHeight;

        transform.position = Vector3.SmoothDamp(
            transform.position, desired, ref velRef, 1f / posSmooth);

        Vector3 lookAt = rearPos + fwd * tpLookAhead;
        Vector3 lookDir = lookAt - transform.position;

        // GUARD: only rotate if we have a valid direction
        if (lookDir.sqrMagnitude > 0.001f)
        {
            Quaternion want = Quaternion.LookRotation(lookDir.normalized, Vector3.up);
            float leanDeg = target.Lean * Mathf.Rad2Deg;
            want = want * Quaternion.Euler(0f, 0f, leanDeg * tpLeanTilt * 0.1f);

            transform.rotation = Quaternion.Slerp(
                transform.rotation, want, rotSmooth * Time.deltaTime);
        }
    }

    // ──────────────────────────────────────────────────────────
    private void UpdateTopDown()
    {
        Vector3 rearPos = target.RearContact;
        Vector3 desired = rearPos + Vector3.up * tdHeight;

        transform.position = Vector3.SmoothDamp(
            transform.position, desired, ref velRef, 1f / posSmooth);

        transform.rotation = Quaternion.Lerp(
            transform.rotation,
            Quaternion.LookRotation(Vector3.down, target.Forward),
            rotSmooth * Time.deltaTime);
    }

    // ──────────────────────────────────────────────────────────
    private void UpdateFirstPerson()
    {
        Vector3 rearPos = target.RearContact
                        + Vector3.up * (target.parameters.rearWheelRadius + fpHeadHeight);

        transform.position = Vector3.SmoothDamp(
            transform.position, rearPos, ref velRef, 1f / posSmooth);

        Quaternion want = Quaternion.LookRotation(target.Forward, Vector3.up);
        float leanDeg = target.Lean * Mathf.Rad2Deg;
        want = want * Quaternion.Euler(0f, 0f, leanDeg);

        transform.rotation = Quaternion.Slerp(
            transform.rotation, want, rotSmooth * Time.deltaTime);
    }

    // ──────────────────────────────────────────────────────────
    private void Update()
    {
        // Cycle camera mode with Tab
        if (Keyboard.current != null && Keyboard.current.tabKey.wasPressedThisFrame)
        {
            mode = (CameraMode)(((int)mode + 1) % 3);
            Debug.Log($"Camera mode: {mode}");
        }
    }
}