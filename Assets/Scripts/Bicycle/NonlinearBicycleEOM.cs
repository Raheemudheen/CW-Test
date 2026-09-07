using UnityEngine;

public static class NonlinearBicycleEOM
{
    public struct BikeState
    {
        public float phi;
        public float delta;
        public float phiDot;
        public float deltaDot;
        public float yawAngle;
        public float yawRate;
        public float rearSpinRate;
        public float frontSpinRate;
        public float speed;
    }

    public struct EOMResult
    {
        public float phiDotDot;
        public float deltaDotDot;
        public float yawRate;
        public float rearSpinRate;
        public float frontSpinRate;
        public bool isValid;
    }

    // ──────────────────────────────────────────────────────────
    public static EOMResult ComputeAccelerations(
        BicycleParameters p,
        BikeState s,
        float leanTorque,
        float steerTorque,
        float gravity = 9.81f)
    {
        EOMResult invalid = new EOMResult { isValid = false };

        // ── Unpack and sanitise inputs immediately ─────────────
        float phi = SanitiseFloat(s.phi, 0f);
        float delta = SanitiseFloat(s.delta, 0f);
        float phiDot = SanitiseFloat(s.phiDot, 0f);
        float dDot = SanitiseFloat(s.deltaDot, 0f);
        float v = SanitiseFloat(s.speed, 0f);
        float g = gravity;

        // ── Parameters ─────────────────────────────────────────
        float w = p.wheelbase;
        float c = p.trail;
        float lam = p.headAngle;
        float rR = p.rearWheelRadius;
        float rF = p.frontWheelRadius;

        float mR = p.rearWheelMass;
        float mF = p.frontWheelMass;
        float mB = p.rearBodyMass;
        float mH = p.frontBodyMass;

        float xB = p.rearBodyCOMx;
        float zB = p.rearBodyCOMz;
        float xH = p.frontBodyCOMx;
        float zH = p.frontBodyCOMz;

        float IRxx = p.rearWheelIxx;
        float IRyy = p.rearWheelIyy;
        float IFxx = p.frontWheelIxx;
        float IFyy = p.frontWheelIyy;
        float IBxx = p.rearBodyIxx;
        float IBxz = p.rearBodyIxz;
        float IBzz = p.rearBodyIzz;
        float IHxx = p.frontBodyIxx;
        float IHxz = p.frontBodyIxz;
        float IHzz = p.frontBodyIzz;

        // ── Clamp delta before computing tan ───────────────────
        // tan(delta) blows up at ±90° — clamp well before that
        delta = Mathf.Clamp(delta, -1.2f, 1.2f);   // ±~69°
        phi = Mathf.Clamp(phi, -1.2f, 1.2f);

        // ── Trig — all safe after clamp ────────────────────────
        float sinPhi = Mathf.Sin(phi);
        float cosPhi = Mathf.Cos(phi);
        float sinDel = Mathf.Sin(delta);
        float cosDel = Mathf.Cos(delta);
        float tanDel = Mathf.Tan(delta);            // safe after clamp
        float sinLam = Mathf.Sin(lam);
        float cosLam = Mathf.Cos(lam);

        // ── Step 1: Yaw rate (nonlinear rolling constraint) ────
        float yawNum = tanDel * cosLam + sinPhi * sinLam * sinDel;
        float yawDenom = w + c * tanDel * cosLam;

        float yawRate = 0f;
        if (Mathf.Abs(yawDenom) > 1e-6f)
            yawRate = v * yawNum / yawDenom;

        yawRate = SanitiseFloat(yawRate, 0f);

        // ── Step 2: Wheel spin rates ───────────────────────────
        float rearSpinRate = (rR > 1e-6f) ? -v / rR : 0f;

        float frontPathSpeed = v;
        if (Mathf.Abs(yawDenom) > 1e-6f)
            frontPathSpeed = v * (1f + (c / w) * tanDel * cosLam);
        float frontSpinRate = (rF > 1e-6f) ? -frontPathSpeed / rF : 0f;

        rearSpinRate = SanitiseFloat(rearSpinRate, 0f);
        frontSpinRate = SanitiseFloat(frontSpinRate, 0f);

        // ── Step 3: Gyroscopic momenta ─────────────────────────
        float SR = IRyy * Mathf.Abs(v / rR);
        float SF = IFyy * Mathf.Abs(frontPathSpeed / rF);

        SR = SanitiseFloat(SR, 0f);
        SF = SanitiseFloat(SF, 0f);

        // ── Step 4: Front assembly properties ─────────────────
        float mA = mH + mF;
        float xA = (mA > 1e-8f) ? (xH * mH + w * mF) / mA : w;
        float zA = (mA > 1e-8f) ? (zH * mH + rF * mF) / mA : rF;
        float uA = (xA - w - c) * cosLam - zA * sinLam;

        float SA = mA * uA + SF * sinLam;

        // ── Step 5: Effective inertias ─────────────────────────
        float Ixx_eff = IRxx + IBxx + IHxx + IFxx
                      + mR * rR * rR
                      + mB * (zB * zB)
                      + mH * (zA * zA)
                      + mF * (rF * rF);

        float Ixz_eff = (IBxz + IHxz) * cosPhi
                      - mB * xB * zB * cosPhi
                      - mH * xH * zA * cosPhi;

        float Izz_steer = IHxx * sinLam * sinLam
                        + IHzz * cosLam * cosLam
                        + IFxx * sinLam * sinLam
                        + mH * (xH - w) * (xH - w) * cosLam * cosLam
                        + mF * c * c * cosLam * cosLam;

        float M00 = Ixx_eff;
        float M01 = Ixz_eff * sinLam
                  + (Izz_steer - c * rF * mA) * cosLam / w
                  + c * SA / w;
        float M10 = M01;
        float M11 = Izz_steer * cosLam * cosLam / (w * w)
                  + SA * c * cosLam / w;

        // Guard against degenerate mass matrix
        M00 = Mathf.Max(M00, 0.01f);
        M11 = Mathf.Max(M11, 0.001f);

        float det = M00 * M11 - M01 * M10;

        if (Mathf.Abs(det) < 1e-9f)
        {
            Debug.LogWarning("NonlinearBicycleEOM: near-singular mass matrix.");
            return invalid;
        }

        // ── Step 6: Gravity forcing ────────────────────────────
        float grav0 = g * sinPhi
                    * (mB * zB + mH * zA + mF * rF + mR * rR);

        float gravSteerNum = mH * (xH - w - c) + mF * (-c);
        float grav1 = (Mathf.Abs(w) > 1e-6f)
                           ? g * sinPhi * cosLam * gravSteerNum / w
                           : 0f;

        // ── Step 7: Gyroscopic forcing ─────────────────────────
        float gyro0 = yawRate * cosPhi * (SR + SF)
                    + dDot * cosLam * SF;

        float gyro1 = -phiDot * cosLam * SF
                    - yawRate * sinPhi * sinLam * SF;

        // ── Step 8: Centrifugal forcing ────────────────────────
        float centrif0 = yawRate * yawRate * cosPhi * sinPhi
                       * (mB * zB * zB + mH * zA * zA
                        + IRxx + IFxx + IBxx + IHxx);

        float centrif1 = 0f; // small, omit for stability

        // ── Step 9: Coriolis ───────────────────────────────────
        float coriolis0 = -2f * phiDot * dDot
                        * (IBxz + IHxz) * sinLam;

        float coriolis1 = phiDot * phiDot
                        * (IBxz + IHxz) * cosLam;

        // ── Step 10: RHS ───────────────────────────────────────
        float rhs0 = leanTorque
                   - grav0
                   + gyro0
                   - centrif0
                   + coriolis0;

        float rhs1 = steerTorque
                   - grav1
                   + gyro1
                   - centrif1
                   + coriolis1;

        // ── Step 11: Solve 2×2 system ──────────────────────────
        float phiDD = (M11 * rhs0 - M01 * rhs1) / det;
        float deltaDD = (M00 * rhs1 - M10 * rhs0) / det;

        // Sanitise outputs — if still NaN something is very wrong
        phiDD = SanitiseFloat(phiDD, 0f);
        deltaDD = SanitiseFloat(deltaDD, 0f);

        // Clamp accelerations to physically reasonable bounds
        phiDD = Mathf.Clamp(phiDD, -50f, 50f);
        deltaDD = Mathf.Clamp(deltaDD, -50f, 50f);

        return new EOMResult
        {
            phiDotDot = phiDD,
            deltaDotDot = deltaDD,
            yawRate = yawRate,
            rearSpinRate = rearSpinRate,
            frontSpinRate = frontSpinRate,
            isValid = true
        };
    }

    // ──────────────────────────────────────────────────────────
    public static float EquilibriumLean(
        BicycleParameters p,
        float speed,
        float delta,
        float gravity = 9.81f)
    {
        delta = Mathf.Clamp(delta, -1.2f, 1.2f);

        float w = p.wheelbase;
        float c = p.trail;
        float cosLam = p.CosHeadAngle;
        float tanDel = Mathf.Tan(delta);
        float denom = w + c * tanDel * cosLam;

        if (Mathf.Abs(denom) < 1e-6f) return 0f;

        float yr = speed * tanDel * cosLam / denom;
        float turnR = (Mathf.Abs(yr) > 1e-6f) ? speed / yr : 1e6f;
        float result = Mathf.Atan2(speed * speed,
                                   gravity * Mathf.Abs(turnR))
                     * Mathf.Sign(delta);

        return SanitiseFloat(result, 0f);
    }

    // ──────────────────────────────────────────────────────────
    /// Returns fallback if value is NaN or Infinity.
    private static float SanitiseFloat(float value, float fallback)
    {
        return (float.IsNaN(value) || float.IsInfinity(value))
               ? fallback
               : value;
    }
}