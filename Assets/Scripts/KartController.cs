using UnityEngine;
using UnityEngine.InputSystem;


[RequireComponent(typeof(Rigidbody))]
public class KartController : MonoBehaviour
{
    [Header("Physics")]
    [SerializeField] private float _gravity = 9.81f;

    [Header("Wheel attachment points")]
    [SerializeField] private Transform _frontLeftWheel;
    [SerializeField] private Transform _frontRightWheel;
    [SerializeField] private Transform _rearLeftWheel;
    [SerializeField] private Transform _rearRightWheel;

    [Header("Weight distribution")]
    [Range(0f, 1f)]
    [SerializeField] private float _frontAxleShare = 0.5f;

    [Header("Steering")]
    [SerializeField] private float _maxSteerAngle = 30f;
    private Quaternion _frontLeftInitialLocalRot;
    private Quaternion _frontRightInitialLocalRot;

    [Header("Input (New Input System)")]
    [SerializeField] private InputActionReference _moveActionRef;
    [SerializeField] private InputActionReference _handbrakeActionRef;

    private float _throttleInput;
    private float _steerInput;
    private bool _handbrakePressed;

    [Header("Engine & drivetrain")]
    [SerializeField] private KartEngine _engine;
    [SerializeField] private float _gearRatio = 8f;
    [SerializeField] private float _drivetrainEfficiency = 0.9f;
    [SerializeField] private float _wheelRadius = 0.3f;

    [Header("Rolling resistance")]
    [SerializeField] private float _rollingResistance = 0.5f;

    [Header("Tyre friction")]
    [SerializeField] private float _frictionCoefficient = 1.0f;
    [SerializeField] private float _lateralStiffness = 80f;

    [Header("Handbrake tuning")]
    [SerializeField] private float _handbrakeRollingMul = 3f;
    [SerializeField] private float _handbrakeRearStiffnessMul = 0.1f;

    private Rigidbody _rb;

    private float _frontLeftNormalForce;
    private float _frontRightNormalForce;
    private float _rearLeftNormalForce;
    private float _rearRightNormalForce;

    private float _sumRearFx;
    private float _sumFrontFy;

    private float _frontLeftVLat;
    private float _frontRightVLat;
    private float _rearLeftVLat;
    private float _rearRightVLat;


    private void Awake()
    {
        _rb = GetComponent<Rigidbody>();
        if (_rb == null)
            Debug.LogError("KartController: Rigidbody not found!");
    }

    private void Start()
    {
        Initialize();
        ComputeStaticWheelLoads();
    }

    private void OnEnable()
    {
        if (_moveActionRef != null && _moveActionRef.action != null)
            _moveActionRef.action.Enable();

        if (_handbrakeActionRef != null && _handbrakeActionRef.action != null)
            _handbrakeActionRef.action.Enable();
    }

    private void OnDisable()
    {
        if (_moveActionRef != null && _moveActionRef.action != null)
            _moveActionRef.action.Disable();

        if (_handbrakeActionRef != null && _handbrakeActionRef.action != null)
            _handbrakeActionRef.action.Disable();
    }

    private void Update()
    {
        ReadInput();
        RotateFrontWheels();
    }

    private void FixedUpdate()
    {
        _sumRearFx = 0f;
        _sumFrontFy = 0f;

        ApplyWheelForces(_frontLeftWheel,  _frontLeftNormalForce,  false, false);
        ApplyWheelForces(_frontRightWheel, _frontRightNormalForce, false, false);
        ApplyWheelForces(_rearLeftWheel,   _rearLeftNormalForce,   true,  true);
        ApplyWheelForces(_rearRightWheel,  _rearRightNormalForce,  true,  true);
    }

    private void Initialize()
    {
        if (_frontLeftWheel != null)
            _frontLeftInitialLocalRot = _frontLeftWheel.localRotation;

        if (_frontRightWheel != null)
            _frontRightInitialLocalRot = _frontRightWheel.localRotation;
    }

    private void ComputeStaticWheelLoads()
    {
        if (_rb == null) return;

        float mass = _rb.mass;
        float totalWeight = mass * _gravity;

        float frontWeight = totalWeight * _frontAxleShare;
        float rearWeight  = totalWeight * (1f - _frontAxleShare);

        _frontLeftNormalForce  = frontWeight * 0.5f;
        _frontRightNormalForce = frontWeight * 0.5f;

        _rearLeftNormalForce   = rearWeight * 0.5f;
        _rearRightNormalForce  = rearWeight * 0.5f;
    }

    private void ReadInput()
    {
        if (_moveActionRef == null || _moveActionRef.action == null)
        {
            _throttleInput = 0f;
            _steerInput = 0f;
            _handbrakePressed = false;
            return;
        }

        Vector2 move = _moveActionRef.action.ReadValue<Vector2>();
        _steerInput    = Mathf.Clamp(move.x, -1f, 1f);
        _throttleInput = Mathf.Clamp(move.y, -1f, 1f);

        if (_handbrakeActionRef != null && _handbrakeActionRef.action != null)
            _handbrakePressed = _handbrakeActionRef.action.IsPressed();
        else
            _handbrakePressed = false;
    }

    private void RotateFrontWheels()
    {
        float steerAngle = _maxSteerAngle * _steerInput;
        Quaternion steerRotation = Quaternion.Euler(0f, steerAngle, 0f);

        if (_frontLeftWheel != null)
            _frontLeftWheel.localRotation = _frontLeftInitialLocalRot * steerRotation;

        if (_frontRightWheel != null)
            _frontRightWheel.localRotation = _frontRightInitialLocalRot * steerRotation;
    }

    private void ApplyWheelForces(Transform wheel, float normalForce, bool isDriven, bool isRear)
    {
        if (wheel == null || _rb == null) return;

        Vector3 wheelPos     = wheel.position;
        Vector3 wheelForward = wheel.forward;
        Vector3 wheelRight   = wheel.right;

        Vector3 v    = _rb.GetPointVelocity(wheelPos);
        float   vLong = Vector3.Dot(v, wheelForward);
        float   vLat  = Vector3.Dot(v, wheelRight);

        float Fx = 0f;
        float Fy = 0f;
        
        if (wheel == _frontLeftWheel)  _frontLeftVLat  = vLat;
        if (wheel == _frontRightWheel) _frontRightVLat = vLat;
        if (wheel == _rearLeftWheel)   _rearLeftVLat   = vLat;
        if (wheel == _rearRightWheel)  _rearRightVLat  = vLat;
        
        if (isDriven && _engine != null)
        {
            Vector3 bodyForward      = transform.forward;
            float   speedAlongForward = Vector3.Dot(_rb.linearVelocity, bodyForward);

            float engineTorque = _engine.Simulate(
                _throttleInput,
                speedAlongForward,
                Time.fixedDeltaTime
            );

            float totalWheelTorque = engineTorque * _gearRatio * _drivetrainEfficiency;
            float wheelTorque      = totalWheelTorque * 0.5f;

            Fx += wheelTorque / _wheelRadius;
        }
        
        float rolling = _rollingResistance;
        if (_handbrakePressed && isRear)
            rolling *= _handbrakeRollingMul;

        Fx += -rolling * vLong;
        
        float latStiff = _lateralStiffness;
        if (_handbrakePressed && isRear)
            latStiff *= _handbrakeRearStiffnessMul;

        Fy += -latStiff * vLat;
        
        float frictionLimit = _frictionCoefficient * normalForce;
        float forceLength   = Mathf.Sqrt(Fx * Fx + Fy * Fy);

        if (forceLength > frictionLimit && forceLength > 1e-6f)
        {
            float scale = frictionLimit / forceLength;
            Fx *= scale;
            Fy *= scale;
        }
        
        if (isRear)   _sumRearFx  += Fx;
        if (!isRear)  _sumFrontFy += Fy;
        
        Vector3 force = wheelForward * Fx + wheelRight * Fy;
        _rb.AddForceAtPosition(force, wheelPos, ForceMode.Force);
    }
    
    private void OnGUI()
    {
        if (_rb == null) return;

        float speedMS  = _rb.linearVelocity.magnitude;
        float speedKmh = speedMS * 3.6f;

        GUI.color = Color.white;
        GUILayout.BeginArea(new Rect(10, 10, 320, 260), GUI.skin.box);

        GUILayout.Label($"Speed: {speedMS:F1} m/s  ({speedKmh:F1} km/h)");
        if (_engine != null)
        {
            GUILayout.Label($"RPM:    {_engine.CurrentRpm:F0}");
            GUILayout.Label($"Torque: {_engine.CurrentTorque:F1} Nm");
        }

        GUILayout.Label($"Rear Fx sum:  {_sumRearFx:F1} N");
        GUILayout.Label($"Front Fy sum: {_sumFrontFy:F1} N");

        GUILayout.Label($"vLat FL: {_frontLeftVLat:F2}");
        GUILayout.Label($"vLat FR: {_frontRightVLat:F2}");
        GUILayout.Label($"vLat RL: {_rearLeftVLat:F2}");
        GUILayout.Label($"vLat RR: {_rearRightVLat:F2}");

        GUILayout.EndArea();
    }
}
