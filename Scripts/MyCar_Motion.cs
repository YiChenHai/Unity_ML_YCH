using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class MyCar_Motion : MonoBehaviour
{
    public enum ControlSource { Agent = 0, Manual = 1 }

    [Header("Control source")]
    public ControlSource controlSource = ControlSource.Agent;

    [Header("Wheel order: FL, RL, RR, FR")]
    public WheelCollider[] wheelColliders = new WheelCollider[4];
    public Transform[] wheelMeshes = new Transform[4];

    [Header("Vehicle geometry (m)")]
    public float wheelBase = 0.76f;   // 轴距 L
    public float trackWidth = 0.47f;  // 轮距 W

    [Header("Control inputs (body frame - Unity标准: X=横向右, Z=前进)")]
    public float vz_input = 0f;  // 前进速度 (Unity Z轴)
    public float vx_input = 0f;  // 横向速度 (Unity X轴)
    public float omega_input = 0f;  // 自转角速度

    [Header("Manual inputs (Inspector)")]
    [Tooltip("前进速度（Unity Z轴），单位 m/s")]
    public float manualVz = 0f;
    [Tooltip("横向速度（Unity X轴，右为正），单位 m/s")]
    public float manualVx = 0f;
    [Tooltip("自转角速度，单位 rad/s")]
    public float manualOmega = 0f;

    [Header("Kinematic scaling & deadzone")]
    public float inputScaleVz = 1f;  // 前进速度缩放
    public float inputScaleVx = 1f;  // 横向速度缩放
    public float inputScaleOmega = 1f;  // 角速度缩放
    public float deadzone = 0.01f;

    [Header("Wheel / Drive")]
    public float maxWheelLinearSpeed = 4.0f;
    public float maxMotorTorque = 200f;
    public float brakeTorqueHigh = 1500f;
    public float brakeGain = 800f;

    [Header("Speed PID (per wheel)")]
    public float speed_Kp = 120f;
    public float speed_Ki = 6f;
    public float speed_Kd = 20f;
    public float speed_integratorLimit = 20f;
    public float speed_outputMin = -200f;
    public float speed_outputMax = 200f;
    public float speedDeadband = 0.02f;

    [Header("Steer PID (per wheel)")]
    public float steer_Kp = 12f;
    public float steer_Ki = 0.5f;
    public float steer_Kd = 2f;
    public float steer_integratorLimit = 10f;
    public float maxSteerRateDeg = 90f;
    public float steerOutputMinDeg = -90f;
    public float steerOutputMaxDeg = 90f;
    public float steerDeadbandDeg = 0.5f;

    [Header("Visual options")]
    public bool forceWheelZto90 = true;

    [Header("Debug")]
    public bool enableDebugLog = true;

    Rigidbody rb;

    private float[] kinSteer = new float[4]; // rad
    private float[] kinSpeed = new float[4]; // m/s

    private float[] appliedSteerDeg = new float[4];
    private float[] appliedSpeed = new float[4];

    private PIDController[] speedPIDs = new PIDController[4];
    private PIDController[] steerPIDs = new PIDController[4];

    private float[] steerCmdDeg = new float[4];
    private float[] wheelSpeedCmd = new float[4];
    private float[] prevWheelSpeedCmd = new float[4]; // 记录前一帧的速度命令

    [HideInInspector] public float[] steerAngles = new float[4];
    [HideInInspector] public float[] wheelSpeeds = new float[4];

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        
        // 启用插值以平滑渲染
        rb.interpolation = RigidbodyInterpolation.Interpolate;

        for (int i = 0; i < 4; i++)
        {
            speedPIDs[i] = new PIDController(speed_Kp, speed_Ki, speed_Kd, -speed_integratorLimit, speed_integratorLimit);
            speedPIDs[i].SetOutputLimits(speed_outputMin, speed_outputMax);

            steerPIDs[i] = new PIDController(steer_Kp, steer_Ki, steer_Kd, -steer_integratorLimit, steer_integratorLimit);
            steerPIDs[i].SetOutputLimits(steerOutputMinDeg, steerOutputMaxDeg);

            steerCmdDeg[i] = 0f;
            wheelSpeedCmd[i] = 0f;
            prevWheelSpeedCmd[i] = 0f;
        }
    }

    void OnValidate()
    {
        for (int i = 0; i < 4; i++)
        {
            if (speedPIDs[i] != null)
            {
                speedPIDs[i].SetGains(speed_Kp, speed_Ki, speed_Kd);
                speedPIDs[i].SetIntegratorLimits(-speed_integratorLimit, speed_integratorLimit);
                speedPIDs[i].SetOutputLimits(speed_outputMin, speed_outputMax);
            }
            if (steerPIDs[i] != null)
            {
                steerPIDs[i].SetGains(steer_Kp, steer_Ki, steer_Kd);
                steerPIDs[i].SetIntegratorLimits(-steer_integratorLimit, steer_integratorLimit);
                steerPIDs[i].SetOutputLimits(steerOutputMinDeg, steerOutputMaxDeg);
            }
        }
    }

    void FixedUpdate()
    {
        float vz, vx, omega;  // Unity标准：vz=前进，vx=横向
        if (controlSource == ControlSource.Agent)
        {
            vz = vz_input * inputScaleVz;
            vx = vx_input * inputScaleVx;
            omega = -omega_input * inputScaleOmega;
        }
        else
        {
            vz = manualVz * inputScaleVz;
            vx = manualVx * inputScaleVx;
            omega = -manualOmega * inputScaleOmega;
        }

        ComputeKinematics(vz, vx, omega);
        MapAndNormalize();
        ApplyPIDControl();
    }

    void LateUpdate()
    {
        UpdateVisualWheels();
    }

    public void SetControl(float vz, float vx, float omega)
    {
        vz_input = vz;  // 前进速度 (Unity Z轴)
        vx_input = vx;  // 横向速度 (Unity X轴)
        omega_input = omega;  // 自转角速度
    }

    void ComputeKinematics(float vz, float vx, float omega)
    {
        if (Mathf.Abs(vz) < deadzone && Mathf.Abs(vx) < deadzone && Mathf.Abs(omega) < deadzone)
        {
            for (int i = 0; i < 4; i++)
            {
                kinSteer[i] = 0f;
                kinSpeed[i] = 0f;
            }
            return;
        }

        Vector2[] wheelPos = new Vector2[4] {
            new Vector2( wheelBase/2f,  trackWidth/2f),  // FL (0)
            new Vector2(-wheelBase/2f,  trackWidth/2f),  // RL (1)
            new Vector2(-wheelBase/2f, -trackWidth/2f),  // RR (2)
            new Vector2( wheelBase/2f, -trackWidth/2f)   // FR (3)
        };

        for (int i = 0; i < 4; i++)
        {
            float vx_rot = -omega * wheelPos[i].y;
            float vz_rot =  omega * wheelPos[i].x;

            float vx_total = vx + vx_rot;
            float vz_total = vz + vz_rot;

            kinSteer[i] = Mathf.Atan2(vx_total, vz_total);  // Unity: atan2(X, Z)
            kinSpeed[i] = Mathf.Sqrt(vx_total * vx_total + vz_total * vz_total);
        }

        for (int i = 0; i < 4; i++)
        {
            if (kinSteer[i] > Mathf.PI / 2f)
            {
                kinSteer[i] -= Mathf.PI;
                kinSpeed[i] = -kinSpeed[i];
            }
            else if (kinSteer[i] < -Mathf.PI / 2f)
            {
                kinSteer[i] += Mathf.PI;
                kinSpeed[i] = -kinSpeed[i];
            }
            kinSteer[i] = -kinSteer[i];
        }

        // 强制左右轮对称性校正 
        //kinSpeed[3] = kinSpeed[0];
        //kinSteer[3] = -kinSteer[0];
        //kinSpeed[2] = kinSpeed[1];
       // kinSteer[2] = -kinSteer[1];

        // 调试打印：每 30 帧打印一次运动学解析结果
        if (enableDebugLog && Time.frameCount % 30 == 0)
        {
        //    Debug.Log($"[ComputeKinematics] Input: vx={vx:F3}, vy={vy:F3}, omega={omega:F3}");
        //    Debug.Log($"[kinSpeed] FL={kinSpeed[0]:F3}, RL={kinSpeed[1]:F3}, RR={kinSpeed[2]:F3}, FR={kinSpeed[3]:F3}");
        //    Debug.Log($"[kinSteer] FL={kinSteer[0]*Mathf.Rad2Deg:F1}°, RL={kinSteer[1]*Mathf.Rad2Deg:F1}°, RR={kinSteer[2]*Mathf.Rad2Deg:F1}°, FR={kinSteer[3]*Mathf.Rad2Deg:F1}°");
        }
    }

    void MapAndNormalize()
    {
        float max_speed_abs = 0f;
        for (int i = 0; i < 4; i++) max_speed_abs = Mathf.Max(max_speed_abs, Mathf.Abs(kinSpeed[i]));
        if (max_speed_abs > maxWheelLinearSpeed && max_speed_abs > 0f)
        {
            float scale = maxWheelLinearSpeed / max_speed_abs;
            for (int i = 0; i < 4; i++) kinSpeed[i] *= scale;
        }

        // 直接映射：kinIdx 顺序与 wheelColliders 顺序一致
        // kinIdx: FL(0), RL(1), RR(2), FR(3)
        // wheelColliders: FL(0), RL(1), RR(2), FR(3)
        for (int i = 0; i < 4; i++)
        {
            appliedSteerDeg[i] = kinSteer[i] * Mathf.Rad2Deg;
            appliedSpeed[i] = kinSpeed[i];
        }

        // 调试打印：映射后的结果
        if (enableDebugLog && Time.frameCount % 30 == 0)
        {
           // Debug.Log($"[appliedSpeed] FL={appliedSpeed[0]:F3}, RL={appliedSpeed[1]:F3}, RR={appliedSpeed[2]:F3}, FR={appliedSpeed[3]:F3}");
            //Debug.Log($"[appliedSteerDeg] FL={appliedSteerDeg[0]:F1}°, RL={appliedSteerDeg[1]:F1}°, RR={appliedSteerDeg[2]:F1}°, FR={appliedSteerDeg[3]:F1}°");
        }
    }

    void ApplyPIDControl()
    {
        float dt = Time.fixedDeltaTime;

        // 检测速度目标是否发生方向反转（从正变负或从负变正）
        bool directionChanged = false;
        for (int j = 0; j < 4; j++)
        {
            float prevSign = Mathf.Sign(prevWheelSpeedCmd[j]);
            float currSign = Mathf.Sign(appliedSpeed[j]);
            
            // 如果前一帧命令和当前目标的符号不同，且都不为零，说明发生了反转
            if (prevSign != 0f && currSign != 0f && prevSign != currSign)
            {
                directionChanged = true;
                break;
            }
        }

        // 方向反转时重置所有轮的 PID 积分器
        if (directionChanged)
        {
            for (int j = 0; j < 4; j++)
            {
                speedPIDs[j].ResetIntegrator();
                steerPIDs[j].ResetIntegrator();
            }
            if (enableDebugLog)
            {
               // Debug.Log("[ApplyPIDControl] Direction reversed! Reset all PID integrators.");
            }
        }

        for (int j = 0; j < 4; j++)
        {
            WheelCollider wc = (wheelColliders != null && j < wheelColliders.Length) ? wheelColliders[j] : null;
            float wheelRadius = (wc != null) ? Mathf.Max(1e-4f, wc.radius) : 0.05f;
            float current_v = 0f;
            if (wc != null) current_v = wc.rpm / 60f * 2f * Mathf.PI * wheelRadius;

            float desired_v = appliedSpeed[j];
            float desiredSteerDeg = appliedSteerDeg[j];

            float speedError = desired_v - current_v;

            // 修复逻辑：分离"停止目标"和"死区控制"
            if (Mathf.Abs(desired_v) < speedDeadband)
            {
                // 目标速度本身接近零 → 完全停止
                speedPIDs[j].ResetIntegrator();
                if (wc != null)
                {
                    wc.motorTorque = 0f;
                    float autoBrake = Mathf.Clamp(brakeGain * Mathf.Abs(current_v), 0f, brakeTorqueHigh);
                    wc.brakeTorque = autoBrake;
                }
                wheelSpeedCmd[j] = 0f;
            }
            else if (Mathf.Abs(speedError) < speedDeadband)
            {
                // 误差在死区内 → 停止加扭矩，但不加制动（让自然摩擦维持）
                speedPIDs[j].ResetIntegrator();
                if (wc != null)
                {
                    wc.motorTorque = 0f;      // 停止加扭矩
                    wc.brakeTorque = 0f;      // 不加制动，避免顿挫！
                }
                wheelSpeedCmd[j] = desired_v;
            }
            else
            {
                // 误差超出死区 → 正常 PID 控制
                float torqueCmd = speedPIDs[j].Update(speedError, dt);
                torqueCmd = Mathf.Clamp(torqueCmd, -maxMotorTorque, maxMotorTorque);
                if (wc != null)
                {
                    wc.brakeTorque = 0f;
                    wc.motorTorque = torqueCmd;
                }
                wheelSpeedCmd[j] = desired_v;
            }

            float steerErrorDeg = Mathf.DeltaAngle(steerCmdDeg[j], desiredSteerDeg);

            float steerRateCmdDeg = 0f;
            if (Mathf.Abs(steerErrorDeg) < steerDeadbandDeg)
            {
                steerPIDs[j].ResetIntegrator();
                steerRateCmdDeg = 0f;
            }
            else
            {
                steerRateCmdDeg = steerPIDs[j].Update(steerErrorDeg, dt);
            }

            steerRateCmdDeg = Mathf.Clamp(steerRateCmdDeg, -maxSteerRateDeg, maxSteerRateDeg);
            steerCmdDeg[j] = Mathf.MoveTowardsAngle(steerCmdDeg[j], steerCmdDeg[j] + steerRateCmdDeg * dt, Mathf.Abs(steerRateCmdDeg) * dt);

            if (wc != null)
            {
                wc.steerAngle = steerCmdDeg[j];
            }

            steerAngles[j] = steerCmdDeg[j] * Mathf.Deg2Rad;
            wheelSpeeds[j] = current_v;
        }

        // 保存当前速度命令用于下一帧比较
        for (int j = 0; j < 4; j++)
        {
            prevWheelSpeedCmd[j] = appliedSpeed[j];
        }

        // 调试打印：实际轮速
        if (enableDebugLog && Time.frameCount % 30 == 0)
        {
          //  Debug.Log($"[wheelSpeeds] FL={wheelSpeeds[0]:F3}, RL={wheelSpeeds[1]:F3}, RR={wheelSpeeds[2]:F3}, FR={wheelSpeeds[3]:F3}");
        }
    }

    void UpdateVisualWheels()
    {
        if (wheelColliders == null || wheelMeshes == null) return;
        for (int i = 0; i < wheelColliders.Length && i < wheelMeshes.Length; i++)
        {
            var wc = wheelColliders[i];
            var mesh = wheelMeshes[i];
            if (wc == null || mesh == null) continue;
            
            Vector3 pos; Quaternion rot;
            wc.GetWorldPose(out pos, out rot);
            mesh.position = pos;

            if (forceWheelZto90)
            {
                Vector3 e = rot.eulerAngles;
                e.z = 90f;
                mesh.rotation = Quaternion.Euler(e);
            }
            else
            {
                mesh.rotation = rot;
            }
        }
    }

    class PIDController
    {
        public float Kp, Ki, Kd;
        private float integrator;
        private float lastError;
        private float integMin = -Mathf.Infinity, integMax = Mathf.Infinity;
        private float outMin = -Mathf.Infinity, outMax = Mathf.Infinity;

        public PIDController(float p, float i, float d, float integMin_, float integMax_)
        {
            Kp = p; Ki = i; Kd = d;
            integrator = 0f; lastError = 0f;
            integMin = integMin_; integMax = integMax_;
        }

        public void SetGains(float p, float i, float d) { Kp = p; Ki = i; Kd = d; }
        public void SetIntegratorLimits(float lo, float hi) { integMin = lo; integMax = hi; }
        public void SetOutputLimits(float lo, float hi) { outMin = lo; outMax = hi; }
        public void ResetIntegrator() { integrator = 0f; lastError = 0f; }

        public float Update(float error, float dt)
        {
            if (dt <= 0f) return 0f;
            integrator = Mathf.Clamp(integrator + error * dt, integMin, integMax);
            float deriv = (error - lastError) / dt;
            lastError = error;
            float outv = Kp * error + Ki * integrator + Kd * deriv;
            outv = Mathf.Clamp(outv, outMin, outMax);
            return outv;
        }
    }
}