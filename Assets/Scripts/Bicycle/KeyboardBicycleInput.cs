using UnityEngine;
using UnityEngine.InputSystem;

public class KeyboardBicycleInput : MonoBehaviour, IBicycleInput
{
    [Header("Smoothing")]
    public float steerSmoothSpeed = 3f;

    private float smoothedSteer;
    private BicycleController bicycle;

    private InputAction steerAction;
    private InputAction accelerateAction;
    private InputAction brakeAction;
    private InputAction resetAction;

    private void Awake()
    {
        bicycle = GetComponent<BicycleController>();
        CreateInputActions();
    }

    private void CreateInputActions()
    {
        steerAction = new InputAction("Steer", InputActionType.Value,
                                      expectedControlType: "Axis");

        // FIX: D/RightArrow = Negative on composite = LEFT steer
        //      A/LeftArrow  = Positive on composite = RIGHT steer
        // Then we NEGATE the result in GetSteerInput()
        // OR simply swap Negative/Positive:
        steerAction.AddCompositeBinding("Axis")
            .With("Negative", "<Keyboard>/a")      // A = left  = negative
            .With("Positive", "<Keyboard>/d");     // D = right = positive

        steerAction.AddCompositeBinding("Axis")
            .With("Negative", "<Keyboard>/leftArrow")
            .With("Positive", "<Keyboard>/rightArrow");

        accelerateAction = new InputAction("Accelerate", InputActionType.Button);
        accelerateAction.AddBinding("<Keyboard>/w");
        accelerateAction.AddBinding("<Keyboard>/upArrow");

        brakeAction = new InputAction("Brake", InputActionType.Button);
        brakeAction.AddBinding("<Keyboard>/s");
        brakeAction.AddBinding("<Keyboard>/downArrow");

        resetAction = new InputAction("Reset", InputActionType.Button);
        resetAction.AddBinding("<Keyboard>/r");

        steerAction.Enable();
        accelerateAction.Enable();
        brakeAction.Enable();
        resetAction.Enable();

        resetAction.performed += _ => bicycle?.ResetBicycle();
    }

    private void OnDestroy()
    {
        steerAction?.Dispose();
        accelerateAction?.Dispose();
        brakeAction?.Dispose();
        resetAction?.Dispose();
    }

    private void OnEnable()
    {
        steerAction?.Enable();
        accelerateAction?.Enable();
        brakeAction?.Enable();
        resetAction?.Enable();
    }

    private void OnDisable()
    {
        steerAction?.Disable();
        accelerateAction?.Disable();
        brakeAction?.Disable();
        resetAction?.Disable();
    }

    public float GetSteerInput()
    {
        // Negate here to fix the inversion:
        // raw positive (D key) → right turn → should be positive steer
        float raw = steerAction.ReadValue<float>();
        smoothedSteer = Mathf.MoveTowards(
            smoothedSteer,
            raw,
            steerSmoothSpeed * Time.fixedDeltaTime
        );
        return smoothedSteer;
    }

    public float GetDriveInput()
    {
        return accelerateAction.ReadValue<float>();
    }

    public float GetBrakeInput()
    {
        return brakeAction.ReadValue<float>();
    }
}