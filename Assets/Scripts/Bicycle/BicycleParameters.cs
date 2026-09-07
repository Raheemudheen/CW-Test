using UnityEngine;

[CreateAssetMenu(fileName = "BicycleParameters",
                 menuName = "Bicycle/Parameters")]
public class BicycleParameters : ScriptableObject
{
    [Header("Geometry")]
    public float wheelbase = 1.02f;
    public float trail = 0.08f;
    public float headAngle = 0.3996f;
    public float rearWheelRadius = 0.3f;
    public float frontWheelRadius = 0.3f;

    [Header("Mass - Rear Frame + Rider (B)")]
    public float rearBodyMass = 85.0f;
    public float rearBodyCOMx = 0.4376f;
    public float rearBodyCOMz = 1.0f;
    public float rearBodyIxx = 9.2f;
    public float rearBodyIxz = 2.4f;
    public float rearBodyIzz = 2.8f;

    [Header("Mass - Front Fork + Handlebar (H)")]
    public float frontBodyMass = 4.0f;
    public float frontBodyCOMx = 0.9f;
    public float frontBodyCOMz = 1.0f;
    public float frontBodyIxx = 0.05892f;
    public float frontBodyIxz = -0.00708f;
    public float frontBodyIzz = 0.00708f;

    [Header("Mass - Rear Wheel (R)")]
    public float rearWheelMass = 2.0f;
    public float rearWheelIyy = 0.06f;
    public float rearWheelIxx = 0.12f;

    [Header("Mass - Front Wheel (F)")]
    public float frontWheelMass = 2.0f;
    public float frontWheelIyy = 0.06f;
    public float frontWheelIxx = 0.12f;

    [Header("Rider Input")]
    public float maxSteerTorque = 10f;
    public float maxLeanTorque = 50f;

    [Header("Damping")]
    public float steerDamping = 3f;
    public float leanDamping = 1f;

    [Header("Balance Controller - Primary")]
    [Tooltip("How strongly to steer toward equilibrium lean")]
    public float balanceLeanGain = 20f;
    [Tooltip("How strongly to damp lean rate")]
    public float balanceLeanRateGain = 8f;
    [Tooltip("How strongly to damp steer rate")]
    public float balanceSteerRateGain = 2f;
    [Tooltip("Speed below which balance controller turns off (m/s)")]
    public float balanceMinSpeed = 0.3f;

    [Header("Balance Controller - Speed Scheduling")]
    [Tooltip("Speed at which gains are at their LOW value (m/s)")]
    public float gainScheduleLowSpeed = 1f;
    [Tooltip("Speed at which gains are at their HIGH value (m/s)")]
    public float gainScheduleHighSpeed = 8f;
    [Tooltip("Gain multiplier at low speed")]
    public float gainMultiplierLow = 2.5f;
    [Tooltip("Gain multiplier at high speed (self-stable range)")]
    public float gainMultiplierHigh = 0.6f;

    [Header("Balance Controller - Limits")]
    [Tooltip("Max steer torque the balance controller can apply (Nm)")]
    public float balanceMaxTorque = 30f;
    [Tooltip("Lean angle beyond which fall is declared (rad)")]
    public float fallThreshold = 1.0f;

    [Header("Speed")]
    public float maxSpeed = 15f;
    public float minSpeedReset = 0.01f;

    // ── Computed ───────────────────────────────────────────────
    public float SinHeadAngle => Mathf.Sin(headAngle);
    public float CosHeadAngle => Mathf.Cos(headAngle);
    public float TotalMass => rearBodyMass + frontBodyMass
                               + rearWheelMass + frontWheelMass;
    public float COMHeight => (rearBodyMass * rearBodyCOMz
                               + frontBodyMass * frontBodyCOMz)
                               / (rearBodyMass + frontBodyMass);

    /// <summary>
    /// Returns a gain multiplier based on current speed.
    /// High multiplier at low speed (needs more help),
    /// low multiplier at high speed (self-stable range needs less).
    /// </summary>
    public float GetSpeedGainMultiplier(float speed)
    {
        float t = Mathf.InverseLerp(
                      gainScheduleLowSpeed,
                      gainScheduleHighSpeed,
                      speed);
        return Mathf.Lerp(gainMultiplierLow, gainMultiplierHigh, t);
    }
}