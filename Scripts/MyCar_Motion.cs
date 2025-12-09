// ...existing code...
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
    public WheelCollider[] wheelColliders = new WheelCollider[4]; // 按顺序赋值：FL, RL, RR, FR
    public Transform[] wheelMeshes = new Transform[4];            // 可视化轮子（可选）

    [Header("Vehicle geometry (m)")]
    public float wheelBase = 0.76f;   // 轴距 L
    public float trackWidth = 0.47f;  // 轮距 W

    [Header("Control inputs (body frame)")]
    // 在车身坐标系中输入：vx 沿 transform.forward 正向（m/s），vy 右为正（m/s），omega 绕 up (rad/s)
    // 当 controlSource==Agent 时，这些由外部 SetControl() 设置；Manual 时使用下面的 manualXXX
    public float vx_input = 0f;
    public float vy_input = 0f;
    public float omega_input = 0f;

    [Header("Manual inputs (Inspector)")]
    [Tooltip("前进速度，单位 m/s")]
    public float manualVx = 0f;
    [Tooltip("横向速度（右为正），单位 m/s")]
    public float manualVy = 0f;
    [Tooltip("自转角速度，单位 rad/s（Inspector 直接输入弧度/秒）")]
    public float manualOmega = 0f;

    [Header("Kinematic scaling & deadzone")]
    public float inputScaleVx = 1f;      // 输入缩放
    public float inputScaleVy = 1f;
    public float inputScaleOmega = 1f;
    public float deadzone = 0.01f;       // 死区

    [Header("Wheel / Drive")]
    public float maxWheelLinearSpeed = 4.0f;  // MAX_MOTOR_SPEED (m/s)
    public float maxMotorTorque = 200f;       // 电机扭矩极限 (N·m)
    public float brakeTorqueHigh = 1500f;
    public float brakeGain = 800f;            // 自动制动力与当前轮速成比例

    [Header("Speed PID (per wheel)")]
    public float speed_Kp = 120f;
    public float speed_Ki = 6f;
    public float speed_Kd = 20f;
    public float speed_integratorLimit = 20f; // anti-windup
    public float speed_outputMin = -200f;
    public float speed_outputMax = 200f;
    public float speedDeadband = 0.02f; // 小误差不触发PID，减少抖动

    [Header("Steer PID (per wheel)")]
    public float steer_Kp = 12f;
    public float steer_Ki = 0.5f;
    public float steer_Kd = 2f;
    public float steer_integratorLimit = 10f;
    public float maxSteerRateDeg = 90f; // 限制舵机角速以避免振荡
    public float steerOutputMinDeg = -90f;
    public float steerOutputMaxDeg = 90f;
    public float steerDeadbandDeg = 0.5f; // 小角度误差不触发PID，减少抖动

    [Header("Visual options")]
    public bool forceWheelZto90 = true;       // 保证可视轮子的世界Z角为90°

    Rigidbody rb;

    // 解析计算出的量（顺序：解析索引 0=FL,1=RL,2=RR,3=FR）
    private float[] kinSteer = new float[4]; // rad
    private float[] kinSpeed = new float[4]; // m/s

    // 应用到 wheelColliders 的映射后命令（按 wheelColliders 顺序）
    private float[] appliedSteerDeg = new float[4]; // deg target
    private float[] appliedSpeed = new float[4];    // m/s target

    // PID 控制器数组
    private PIDController[] speedPIDs = new PIDController[4];
    private PIDController[] steerPIDs = new PIDController[4];

    // 当前舵机角度命令（deg）和当前速度命令缓存（m/s）
    private float[] steerCmdDeg = new float[4];
    private float[] wheelSpeedCmd = new float[4];

    // 输出公开（按 wheelColliders 顺序）
    [HideInInspector] public float[] steerAngles = new float[4]; // rad
    [HideInInspector] public float[] wheelSpeeds = new float[4]; // m/s (当前实际轮线速)

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        for (int i = 0; i < 4; i++)
        {
            speedPIDs[i] = new PIDController(speed_Kp, speed_Ki, speed_Kd, -speed_integratorLimit, speed_integratorLimit);
            speedPIDs[i].SetOutputLimits(speed_outputMin, speed_outputMax);

            steerPIDs[i] = new PIDController(steer_Kp, steer_Ki, steer_Kd, -steer_integratorLimit, steer_integratorLimit);
            steerPIDs[i].SetOutputLimits(steerOutputMinDeg, steerOutputMaxDeg);

            steerCmdDeg[i] = 0f;
            wheelSpeedCmd[i] = 0f;
        }
    }

    void OnValidate()
    {
        // 当在 Editor 修改 PID 参数时更新控制器参数
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
        // 依据控制源选择三速度
        float vx, vy, omega;
        if (controlSource == ControlSource.Agent)
        {
            vx = vx_input * inputScaleVx;
            vy = vy_input * inputScaleVy;
            // 把正的 omega 取反，使正值表示顺时针（车头向右）旋转
            omega = -omega_input * inputScaleOmega;
        }
        else // Manual: 从 Inspector 手动输入，manualOmega 单位为 rad/s
        {
            vx = manualVx * inputScaleVx;
            vy = manualVy * inputScaleVy;
            // Manual 模式也统一取反
            omega = -manualOmega * inputScaleOmega;
        }

        ComputeKinematics(vx, vy, omega);
        MapAndNormalize();
        ApplyPIDControl();
        UpdateVisualWheels();
    }

    // 外部接口：设置控制量（body frame），Agent 使用此函数下发控制
    public void SetControl(float vx, float vy, float omega)
    {
        vx_input = vx;
        vy_input = vy;
        omega_input = omega;
    }

    // 1. 运动学计算（按 txt）
    void ComputeKinematics(float vx, float vy, float omega)
    {
        // 如果都在死区，清零解析输出并返回
        if (Mathf.Abs(vx) < deadzone && Mathf.Abs(vy) < deadzone && Mathf.Abs(omega) < deadzone)
        {
            for (int i = 0; i < 4; i++)
            {
                kinSteer[i] = 0f;
                kinSpeed[i] = 0f;
            }
            return;
        }

        Vector2[] wheelPos = new Vector2[4] {
            new Vector2( wheelBase/2f,  trackWidth/2f),  // FL
            new Vector2(-wheelBase/2f,  trackWidth/2f),  // RL
            new Vector2(-wheelBase/2f, -trackWidth/2f),  // RR
            new Vector2( wheelBase/2f, -trackWidth/2f)   // FR
        };

        for (int i = 0; i < 4; i++)
        {
            float vx_rot = -omega * wheelPos[i].y;  // -ω * y
            float vy_rot =  omega * wheelPos[i].x;  //  ω * x

            float vx_total = vx + vx_rot;
            float vy_total = vy + vy_rot;

            kinSteer[i] = Mathf.Atan2(vy_total, vx_total); // rad
            kinSpeed[i] = Mathf.Sqrt(vx_total * vx_total + vy_total * vy_total); // m/s
        }

        // 将 steer 限制到 [-pi/2, pi/2]，并在需要时翻转速度符号
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
        }
    }

    // 2. 归一化并映射到实际 wheelCollider 索引（执行你之前要求的前后互换）
    void MapAndNormalize()
    {
        // 速度归一化（若某轮超过最大速度则按比例缩放）
        float max_speed_abs = 0f;
        for (int i = 0; i < 4; i++) max_speed_abs = Mathf.Max(max_speed_abs, Mathf.Abs(kinSpeed[i]));
        if (max_speed_abs > maxWheelLinearSpeed && max_speed_abs > 0f)
        {
            float scale = maxWheelLinearSpeed / max_speed_abs;
            for (int i = 0; i < 4; i++) kinSpeed[i] *= scale;
        }

        // 解析索引 -> wheelColliders 索引 映射：把前左与后左互換，右侧同理
        // kin idx: 0=FL,1=RL,2=RR,3=FR
        int[] map = new int[4] { 1, 0, 3, 2 }; // kin0->1, kin1->0, kin2->3, kin3->2

        for (int i = 0; i < 4; i++)
        {
            appliedSteerDeg[i] = 0f;
            appliedSpeed[i] = 0f;
        }
        for (int kinIdx = 0; kinIdx < 4; kinIdx++)
        {
            int tgt = map[kinIdx];
            appliedSteerDeg[tgt] = kinSteer[kinIdx] * Mathf.Rad2Deg;
            appliedSpeed[tgt] = kinSpeed[kinIdx];
        }
    }

    // 3. 应用 PID：速度 PID 输出 motorTorque，转向 PID 输出舵机角速并积分到 steerCmdDeg
    void ApplyPIDControl()
    {
        float dt = Time.fixedDeltaTime;

        for (int j = 0; j < 4; j++)
        {
            WheelCollider wc = (wheelColliders != null && j < wheelColliders.Length) ? wheelColliders[j] : null;
            float wheelRadius = (wc != null) ? Mathf.Max(1e-4f, wc.radius) : 0.05f;
            float current_v = 0f;
            if (wc != null) current_v = wc.rpm / 60f * 2f * Mathf.PI * wheelRadius;

            float desired_v = appliedSpeed[j];
            float desiredSteerDeg = appliedSteerDeg[j];

            // --- 速度 PID -> 输出扭矩 (N·m)
            float speedError = desired_v - current_v;

            if (Mathf.Abs(desired_v) < speedDeadband || Mathf.Abs(speedError) < speedDeadband)
            {
                // 认为目标为 0 或误差极小：施加制动并重置积分
                speedPIDs[j].ResetIntegrator();
                if (wc != null)
                {
                    wc.motorTorque = 0f;
                    float autoBrake = Mathf.Clamp(brakeGain * Mathf.Abs(current_v), 0f, brakeTorqueHigh);
                    wc.brakeTorque = autoBrake;
                }
                wheelSpeedCmd[j] = 0f;
            }
            else
            {
                // PID 控制输出直接作为 motorTorque（或作为前馈+PID）
                float torqueCmd = speedPIDs[j].Update(speedError, dt);
                torqueCmd = Mathf.Clamp(torqueCmd, -maxMotorTorque, maxMotorTorque);
                if (wc != null)
                {
                    wc.brakeTorque = 0f;
                    wc.motorTorque = torqueCmd;
                }
                wheelSpeedCmd[j] = desired_v;
            }

            // --- 转向 PID -> 输出角速度 deg/s，积分到 steerCmdDeg（模拟舵机）
            // 计算最短角误差（deg）
            float steerErrorDeg = Mathf.DeltaAngle(steerCmdDeg[j], desiredSteerDeg);

            float steerRateCmdDeg = 0f;
            if (Mathf.Abs(steerErrorDeg) < steerDeadbandDeg)
            {
                // 误差极小，不触发 PID，清积分
                steerPIDs[j].ResetIntegrator();
                steerRateCmdDeg = 0f;
            }
            else
            {
                steerRateCmdDeg = steerPIDs[j].Update(steerErrorDeg, dt);
            }

            steerRateCmdDeg = Mathf.Clamp(steerRateCmdDeg, -maxSteerRateDeg, maxSteerRateDeg);

            // 集成成当前舵机角度，使用 MoveTowardsAngle 保持数值稳定
            steerCmdDeg[j] = Mathf.MoveTowardsAngle(steerCmdDeg[j], steerCmdDeg[j] + steerRateCmdDeg * dt, Mathf.Abs(steerRateCmdDeg) * dt);

            if (wc != null)
            {
                wc.steerAngle = steerCmdDeg[j];
            }

            // 供外部读取：实际应用到物理轮子的角度/速度
            steerAngles[j] = steerCmdDeg[j] * Mathf.Deg2Rad;
            wheelSpeeds[j] = current_v;
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

    // 简单 PID 控制器实现（带输出限幅和积分限幅）
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

        // error: 系统误差 (setpoint - measurement)
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
// ...existing code...