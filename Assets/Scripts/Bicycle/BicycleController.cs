using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class BicycleController : MonoBehaviour
{
    // ── Inspector ──────────────────────────────────────────────
    [Header("Parameters")]
    public BicycleParameters parameters;

    [Header("References")]
    public Transform rearWheelTransform;
    public Transform frameTransform;
    public Transform forkTransform;
    public Transform frontWheelTransform;

    [Header("Ground")]
    public LayerMask groundLayer = ~0;
    public float groundRayLength = 2f;

    // ── State Machine ──────────────────────────────────────────
    public enum BikeState { Riding, Fallen, Resetting }

    [Header("Runtime State (Read Only)")]
    [SerializeField] private BikeState bikeState = BikeState.Riding;
    [SerializeField] private float phi;
    [SerializeField] private float delta;
    [SerializeField] private float phiDot;
    [SerializeField] private float deltaDot;
    [SerializeField] private float yawAngle;
    [SerializeField] private float yawRate;
    [SerializeField] private float speed;
    [SerializeField] private float speedKmh;
    [SerializeField] private float rearSpinAngle;
    [SerializeField] private float frontSpinAngle;
    [SerializeField] private float rearSpinRate;
    [SerializeField] private float frontSpinRate;
    [SerializeField] private float phiDotDot;
    [SerializeField] private float deltaDotDot;
    [SerializeField] private float equilibriumLean;
    [SerializeField] private float balanceTorqueApplied;
    [SerializeField] private float gainMultiplier;
    [SerializeField] private bool isGrounded;

    // ── Internal ───────────────────────────────────────────────
    private Vector3 rearContactPoint;
    private IBicycleInput inputHandler;
    private Rigidbody rb;
    private float resetTimer;
    private const float ResetDuration = 0.3f;

    // ── Public Accessors ───────────────────────────────────────
    public BikeState State => bikeState;
    public float Lean => phi;
    public float Steer => delta;
    public float Speed => speed;
    public float SpeedKmh => speedKmh;
    public bool IsGrounded => isGrounded;
    public float YawAngle => yawAngle;
    public float EquilibriumLean => equilibriumLean;
    public float GainMultiplier => gainMultiplier;
    public Vector3 RearContact => rearContactPoint;
    public Vector3 Forward =>
        new Vector3(Mathf.Sin(yawAngle), 0f, Mathf.Cos(yawAngle));

    // ──────────────────────────────────────────────────────────
    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.isKinematic = true;

        inputHandler = GetComponent<IBicycleInput>();
        if (inputHandler == null)
            inputHandler = gameObject.AddComponent<KeyboardBicycleInput>();
    }

    private void Start()
    {
        if (parameters == null)
        {
            Debug.LogError("BicycleController: No BicycleParameters assigned!");
            enabled = false;
            return;
        }

        rearContactPoint = new Vector3(
            transform.position.x, 0f, transform.position.z);
        yawAngle = transform.eulerAngles.y * Mathf.Deg2Rad;

        SnapRootToGround();
        UpdateChildTransforms();

        Debug.Log("Nonlinear Whipple Bicycle initialised.");
        Debug.Log("Accelerate with W. Steer with A/D. Reset with R.");
    }

    // ──────────────────────────────────────────────────────────
    private void FixedUpdate()
    {
        switch (bikeState)
        {
            case BikeState.Riding: UpdateRiding(); break;
            case BikeState.Fallen: break;
            case BikeState.Resetting: UpdateResetting(); break;
        }
    }

    // ──────────────────────────────────────────────────────────
    private void UpdateRiding()
    {
        float dt = Time.fixedDeltaTime;

        // ── 1. Ground detection ────────────────────────────────
        isGrounded = CheckGrounded(out float groundY);

        // ── 2. Input ───────────────────────────────────────────
        float steerInput = inputHandler.GetSteerInput();
        float driveInput = inputHandler.GetDriveInput();
        float brakeInput = inputHandler.GetBrakeInput();

        // ── 3. Speed ───────────────────────────────────────────
        float driveForce = driveInput * 40f;
        float brakeForce = brakeInput * 70f;
        float dragForce = speed * speed * 0.025f + speed * 0.2f;
        float netForce = driveForce - brakeForce - dragForce;

        speed += (netForce / parameters.TotalMass) * dt;
        speed = Mathf.Clamp(speed, 0f, parameters.maxSpeed);
        speedKmh = speed * 3.6f;

        // ── 4. Compute equilibrium lean ────────────────────────
        // What lean angle is needed to sustain THIS speed + steer?
        equilibriumLean = NonlinearBicycleEOM.EquilibriumLean(
            parameters, speed, delta);

        // ── 5. Speed-scheduled gain ────────────────────────────
        // At low speeds we need aggressive correction.
        // In the self-stable range (~4–9 m/s) we need less.
        gainMultiplier = parameters.GetSpeedGainMultiplier(speed);

        // ── 6. Cascade balance controller ─────────────────────
        float leanError = phi - equilibriumLean;
        float leanRateError = phiDot;

        // Primary: steer to correct lean error
        float primaryTorque = -parameters.balanceLeanGain
                            * gainMultiplier
                            * leanError;

        // Derivative: damp lean rate to prevent overshoot
        float derivTorque = -parameters.balanceLeanRateGain
                            * gainMultiplier
                            * leanRateError;

        // Steer rate damping: prevents handlebar oscillation
        float steerRateDamp = -parameters.balanceSteerRateGain
                            * deltaDot;

        float rawBalanceTorque = primaryTorque + derivTorque + steerRateDamp;

        // Clamp balance torque — the rider can only apply so much
        balanceTorqueApplied = Mathf.Clamp(
            rawBalanceTorque,
           -parameters.balanceMaxTorque,
            parameters.balanceMaxTorque);

        // Scale down balance when speed is very low — below balanceMinSpeed
        // the bike cannot self-balance regardless, so we fade out gracefully
        float speedFade = Mathf.InverseLerp(
                              0f,
                              parameters.balanceMinSpeed,
                              speed);
        balanceTorqueApplied *= speedFade;

        // ── 7. Total steer torque ──────────────────────────────
        // Rider input + balance + steer damping
        float riderTorque = steerInput * parameters.maxSteerTorque;
        float steerDampTorque = -parameters.steerDamping * deltaDot;
        float totalSteerTorque = riderTorque
                               + balanceTorqueApplied
                               + steerDampTorque;

        // ── 8. Pack state ──────────────────────────────────────
        NonlinearBicycleEOM.BikeState state = new NonlinearBicycleEOM.BikeState
        {
            phi = phi,
            delta = delta,
            phiDot = phiDot,
            deltaDot = deltaDot,
            yawAngle = yawAngle,
            yawRate = yawRate,
            speed = speed,
            rearSpinRate = rearSpinRate,
            frontSpinRate = frontSpinRate
        };

        // ── 9. Evaluate nonlinear EOM ──────────────────────────
        NonlinearBicycleEOM.EOMResult result =
            NonlinearBicycleEOM.ComputeAccelerations(
                parameters,
                state,
                leanTorque: 0f,
                steerTorque: totalSteerTorque
            );

        if (!result.isValid)
        {
            Debug.LogWarning("EOM invalid — skipping frame.");
            return;
        }

        phiDotDot = result.phiDotDot;
        deltaDotDot = result.deltaDotDot;
        yawRate = result.yawRate;
        rearSpinRate = result.rearSpinRate;
        frontSpinRate = result.frontSpinRate;

        // ── 10. RK4 integration ────────────────────────────────
        IntegrateRK4(dt, totalSteerTorque);

        // ── 11. Fall check ─────────────────────────────────────
        if (Mathf.Abs(phi) > parameters.fallThreshold)
        {
            TransitionToFallen();
            return;
        }

        // ── 12. Advance position ───────────────────────────────
        rearContactPoint += Forward * (speed * dt);
        rearContactPoint.y = isGrounded ? groundY : 0f;

        // ── 13. Apply transforms ───────────────────────────────
        SnapRootToGround();
        UpdateChildTransforms();
    }

    // ──────────────────────────────────────────────────────────
    private void IntegrateRK4(float dt, float totalSteerTorque)
    {
        float[] y = new float[]
        {
            phi, phiDot, delta, deltaDot, yawAngle
        };

        float[] k1 = Derivatives(y, totalSteerTorque);
        float[] k2 = Derivatives(AddScaled(y, k1, dt * 0.5f), totalSteerTorque);
        float[] k3 = Derivatives(AddScaled(y, k2, dt * 0.5f), totalSteerTorque);
        float[] k4 = Derivatives(AddScaled(y, k3, dt * 1.0f), totalSteerTorque);

        for (int i = 0; i < y.Length; i++)
            y[i] += (dt / 6f) * (k1[i] + 2f * k2[i] + 2f * k3[i] + k4[i]);

        phi = y[0];
        phiDot = y[1];
        delta = y[2];
        deltaDot = y[3];
        yawAngle = y[4];

        SanitiseState();

        phi = Mathf.Clamp(phi, -Mathf.PI * 0.48f, Mathf.PI * 0.48f);
        delta = Mathf.Clamp(delta, -Mathf.PI / 3f, Mathf.PI / 3f);

        rearSpinAngle += rearSpinRate * dt;
        frontSpinAngle += frontSpinRate * dt;
    }

    // ──────────────────────────────────────────────────────────
    private float[] Derivatives(float[] y, float totalSteerTorque)
    {
        float _phi = y[0];
        float _phiDot = y[1];
        float _delta = y[2];
        float _deltaDot = y[3];
        float _yaw = y[4];

        NonlinearBicycleEOM.BikeState st = new NonlinearBicycleEOM.BikeState
        {
            phi = _phi,
            delta = _delta,
            phiDot = _phiDot,
            deltaDot = _deltaDot,
            yawAngle = _yaw,
            yawRate = yawRate,
            speed = speed,
            rearSpinRate = rearSpinRate,
            frontSpinRate = frontSpinRate
        };

        NonlinearBicycleEOM.EOMResult res =
            NonlinearBicycleEOM.ComputeAccelerations(
                parameters,
                st,
                leanTorque: 0f,
                steerTorque: totalSteerTorque
            );

        // Recompute yaw rate at this sub-state
        float w = parameters.wheelbase;
        float c = parameters.trail;
        float cosLam = parameters.CosHeadAngle;
        float sinLam = parameters.SinHeadAngle;
        float clampedDelta = Mathf.Clamp(_delta, -1.2f, 1.2f);
        float tanDel = Mathf.Tan(clampedDelta);
        float sinPhi = Mathf.Sin(_phi);
        float num = tanDel * cosLam + sinPhi * sinLam * Mathf.Sin(clampedDelta);
        float denom = w + c * tanDel * cosLam;
        float _yawRate = (Mathf.Abs(denom) > 1e-4f)
                       ? speed * num / denom
                       : 0f;

        if (!res.isValid)
        {
            // Return zero derivatives — better than NaN propagation
            return new float[] { _phiDot, 0f, _deltaDot, 0f, _yawRate };
        }

        return new float[]
        {
            _phiDot,
            res.phiDotDot,
            _deltaDot,
            res.deltaDotDot,
            _yawRate
        };
    }

    // ──────────────────────────────────────────────────────────
    private float[] AddScaled(float[] y, float[] k, float scale)
    {
        float[] result = new float[y.Length];
        for (int i = 0; i < y.Length; i++)
            result[i] = y[i] + scale * k[i];
        return result;
    }

    // ──────────────────────────────────────────────────────────
    private void SanitiseState()
    {
        phi = Sanitise(phi, 0f);
        phiDot = Sanitise(phiDot, 0f);
        delta = Sanitise(delta, 0f);
        deltaDot = Sanitise(deltaDot, 0f);
        yawAngle = Sanitise(yawAngle, 0f);
        yawRate = Sanitise(yawRate, 0f);
        speed = Sanitise(speed, 0f);
        rearSpinAngle = Sanitise(rearSpinAngle, 0f);
        frontSpinAngle = Sanitise(frontSpinAngle, 0f);
        rearSpinRate = Sanitise(rearSpinRate, 0f);
        frontSpinRate = Sanitise(frontSpinRate, 0f);
    }

    private float Sanitise(float v, float fallback) =>
        (float.IsNaN(v) || float.IsInfinity(v)) ? fallback : v;

    // ──────────────────────────────────────────────────────────
    private void UpdateResetting()
    {
        resetTimer -= Time.fixedDeltaTime;
        if (resetTimer <= 0f)
        {
            bikeState = BikeState.Riding;
            Debug.Log("Bicycle ready.");
        }
    }

    // ──────────────────────────────────────────────────────────
    private bool CheckGrounded(out float groundY)
    {
        Vector3 rayOrigin = rearContactPoint + Vector3.up * 0.5f;
        if (Physics.Raycast(rayOrigin, Vector3.down, out RaycastHit hit,
                            groundRayLength, groundLayer))
        {
            groundY = hit.point.y;
            return true;
        }
        groundY = 0f;
        return false;
    }

    // ──────────────────────────────────────────────────────────
    private void SnapRootToGround()
    {
        transform.position = rearContactPoint
                           + Vector3.up * parameters.rearWheelRadius;
        transform.rotation = Quaternion.Euler(
                                 0f, yawAngle * Mathf.Rad2Deg, 0f);
    }

    // ──────────────────────────────────────────────────────────
    private void UpdateChildTransforms()
    {
        if (float.IsNaN(phi) || float.IsNaN(delta) ||
            float.IsNaN(rearSpinAngle) || float.IsNaN(frontSpinAngle))
        {
            SanitiseState();
            return;
        }

        float rR = parameters.rearWheelRadius;
        float rF = parameters.frontWheelRadius;
        float wb = parameters.wheelbase;

        Quaternion leanQ = Quaternion.AngleAxis(
                                 phi * Mathf.Rad2Deg, Vector3.forward);
        Quaternion steerQ = Quaternion.AngleAxis(
                                -delta * Mathf.Rad2Deg, Vector3.up);
        Quaternion rearSpinQ = Quaternion.AngleAxis(
                                    rearSpinAngle * Mathf.Rad2Deg, Vector3.right);
        Quaternion frontSpinQ = Quaternion.AngleAxis(
                                    frontSpinAngle * Mathf.Rad2Deg, Vector3.right);

        if (rearWheelTransform != null)
        {
            rearWheelTransform.localPosition = Vector3.zero;
            rearWheelTransform.localRotation = leanQ * rearSpinQ;
        }

        if (frameTransform != null)
        {
            float frameUp = parameters.rearBodyCOMz - rR;
            Vector3 frameOffset = leanQ * new Vector3(0f, frameUp, wb * 0.5f);
            frameTransform.localPosition = frameOffset;
            frameTransform.localRotation = leanQ;
        }

        if (forkTransform != null)
        {
            float axleHeightDiff = rF - rR;
            Vector3 forkOffset = leanQ * new Vector3(0f, axleHeightDiff, wb);
            forkTransform.localPosition = forkOffset;
            forkTransform.localRotation = leanQ * steerQ;
        }

        if (frontWheelTransform != null)
        {
            frontWheelTransform.localPosition = Vector3.zero;
            frontWheelTransform.localRotation = frontSpinQ;
        }
    }

    // ──────────────────────────────────────────────────────────
    private void TransitionToFallen()
    {
        bikeState = BikeState.Fallen;
        speed = 0f;
        Debug.Log($"Fell! Lean={phi * Mathf.Rad2Deg:F1}° " +
                  $"Steer={delta * Mathf.Rad2Deg:F1}° " +
                  $"Speed={speedKmh:F1}km/h. Press R.");
    }

    // ──────────────────────────────────────────────────────────
    public void ResetBicycle()
    {
        phi = 0f;
        delta = 0f;
        phiDot = 0f;
        deltaDot = 0f;
        speed = 0f;
        speedKmh = 0f;
        yawRate = 0f;
        phiDotDot = 0f;
        deltaDotDot = 0f;
        rearSpinRate = 0f;
        frontSpinRate = 0f;
        rearSpinAngle = 0f;
        frontSpinAngle = 0f;
        equilibriumLean = 0f;
        balanceTorqueApplied = 0f;

        rearContactPoint = new Vector3(
            transform.position.x, 0f, transform.position.z);
        yawAngle = transform.eulerAngles.y * Mathf.Deg2Rad;

        bikeState = BikeState.Resetting;
        resetTimer = ResetDuration;

        SnapRootToGround();
        UpdateChildTransforms();

        Debug.Log("Bicycle reset.");
    }
}