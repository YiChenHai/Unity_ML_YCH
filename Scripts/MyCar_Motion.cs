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
    public float deadzone = 0.02f;

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
    public float speedDeadband = 0.01f;

    [Header("Steer PID (per wheel)")]
    public float steer_Kp = 40f;  // 增大比例增益，大误差时响应更快
    public float steer_Ki = 2f;   // 增大积分增益，消除稳态误差
    public float steer_Kd = 5f;   // 增大微分增益，减少超调
    public float steer_integratorLimit = 10f;
    public float maxSteerRateDeg = 360f;  // 最大转向角速度（度/秒）
    public float steerDeadbandDeg = 0.2f;

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
    private float[] prevWheelSpeedCmd = new float[4]; // 记录前一帧的速度命令

    [HideInInspector] public float[] steerAngles = new float[4];
    [HideInInspector] public float[] wheelSpeeds = new float[4];
    [HideInInspector] public bool flipOccurred = false;  // 本帧是否发生翻转

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
            steerPIDs[i].SetOutputLimits(-maxSteerRateDeg, maxSteerRateDeg);  // PID输出为角速率（度/秒）

            steerCmdDeg[i] = 0f;
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
                steerPIDs[i].SetOutputLimits(-maxSteerRateDeg, maxSteerRateDeg);  // PID输出为角速率（度/秒）
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

    /// <summary>
    /// 运动学求解：根据车体速度(vz, vx, omega)计算四个轮子的转向角和轮速
    /// 核心原理：刚体运动学 - 每个轮子的速度 = 车体平移速度 + 旋转产生的切向速度
    /// </summary>
    void ComputeKinematics(float vz, float vx, float omega)
    {
        // 重置翻转标志
        flipOccurred = false;
        
        // 死区处理：所有输入接近零时，停止所有轮子
        if (Mathf.Abs(vz) < deadzone && Mathf.Abs(vx) < deadzone && Mathf.Abs(omega) < deadzone)
        {
            for (int i = 0; i < 4; i++)
            {
                kinSteer[i] = 0f;
                kinSpeed[i] = 0f;
            }
            return;
        }

        // 定义四个轮子在车体坐标系中的位置 (x=纵向, y=横向)
        // Unity坐标系：X轴=右，Z轴=前
        Vector2[] wheelPos = new Vector2[4] {
            new Vector2( wheelBase/2f,  trackWidth/2f),  // FL (0): 前左轮
            new Vector2(-wheelBase/2f,  trackWidth/2f),  // RL (1): 后左轮
            new Vector2(-wheelBase/2f, -trackWidth/2f),  // RR (2): 后右轮
            new Vector2( wheelBase/2f, -trackWidth/2f)   // FR (3): 前右轮
        };

        // 第一步：速度合成 - 刚体运动学公式
        // v_wheel = v_body + omega × r （叉乘）
        for (int i = 0; i < 4; i++)
        {
            // 旋转产生的切向速度分量（2D叉乘简化形式） 
            // 标准公式：v_x = omega * y, v_z = omega * x
            // wheelPos.x = 轮子在纵向(Z轴)的位置, wheelPos.y = 轮子在横向(X轴)的位置
            float vz_rot =  omega * wheelPos[i].y;  // 纵向分量 (前后方向)
            float vx_rot =  omega * wheelPos[i].x;  // 横向分量 (左右方向)

            // 合成总速度：平移速度 + 旋转速度
            float vz_total = vz + vz_rot;
            float vx_total = vx + vx_rot;

            // 第二步：转换为极坐标（速度向量 → 转向角 + 轮速标量）
            kinSteer[i] = Mathf.Atan2(vx_total, vz_total);  // Unity: atan2(X横向, Z纵向)
            kinSpeed[i] = Mathf.Sqrt(vx_total * vx_total + vz_total * vz_total);
            
            // 第三步：角度折叠到±90°范围（舵轮物理限制）
            float angleDeg = kinSteer[i] * Mathf.Rad2Deg;
            if (angleDeg > 90f)
            {
                flipOccurred = true;  // 标记发生翻转
                Debug.Log($"[Flip] Wheel {i}: angle {angleDeg:F1}° > 90°, flipped to {angleDeg - 180f:F1}°, speed reversed");
                kinSteer[i] = (angleDeg - 180f) * Mathf.Deg2Rad;
                kinSpeed[i] = -kinSpeed[i];  // 速度反向
            }
            else if (angleDeg < -90f)
            {
                flipOccurred = true;  // 标记发生翻转
                Debug.Log($"[Flip] Wheel {i}: angle {angleDeg:F1}° < -90°, flipped to {angleDeg + 180f:F1}°, speed reversed");
                kinSteer[i] = (angleDeg + 180f) * Mathf.Deg2Rad;
                kinSpeed[i] = -kinSpeed[i];  // 速度反向
            }
        }

        // 调试日志：每 30 帧打印一次运动学解析结果
        if (enableDebugLog && Time.frameCount % 30 == 0)
        {
            //Debug.Log($"[ComputeKinematics] Input: vz={vz:F3}, vx={vx:F3}, omega={omega:F3}");
            //Debug.Log($"[kinSpeed] FL={kinSpeed[0]:F3}, RL={kinSpeed[1]:F3}, RR={kinSpeed[2]:F3}, FR={kinSpeed[3]:F3}");
           // Debug.Log($"[kinSteer] FL={kinSteer[0]*Mathf.Rad2Deg:F1}°, RL={kinSteer[1]*Mathf.Rad2Deg:F1}°, RR={kinSteer[2]*Mathf.Rad2Deg:F1}°, FR={kinSteer[3]*Mathf.Rad2Deg:F1}°");
            Debug.Log($"[steerCmdDeg] FL={steerCmdDeg[0]:F1}°, RL={steerCmdDeg[1]:F1}°, RR={steerCmdDeg[2]:F1}°, FR={steerCmdDeg[3]:F1}°");
        }
    }
/// <summary>
    /// 速度归一化和映射：确保轮速不超过物理限制，并映射到实际轮子数组
    /// </summary>
    void MapAndNormalize()
    {
        // 第一步：全局速度限幅 - 找出最大轮速
        float max_speed_abs = 0f;
        for (int i = 0; i < 4; i++) max_speed_abs = Mathf.Max(max_speed_abs, Mathf.Abs(kinSpeed[i]));
        
        // 如果最大轮速超过限制，按比例缩放所有轮子（保持相对速度比例）
        if (max_speed_abs > maxWheelLinearSpeed && max_speed_abs > 0f)
        {
            float scale = maxWheelLinearSpeed / max_speed_abs;
            for (int i = 0; i < 4; i++) kinSpeed[i] *= scale;
        }

        // 第二步：直接映射到 wheelColliders 数组（索引顺序一致）
        // kinIdx 顺序：FL(0), RL(1), RR(2), FR(3)
        // wheelColliders 顺序：FL(0), RL(1), RR(2), FR(3)
        for (int i = 0; i < 4; i++)
        {
            appliedSteerDeg[i] = kinSteer[i] * Mathf.Rad2Deg;  // 弧度→角度
            appliedSpeed[i] = kinSpeed[i];                      // 速度直接赋值
        }

        // 调试打印：映射后的结果
        if (enableDebugLog && Time.frameCount % 30 == 0)
        {
           // Debug.Log($"[appliedSpeed] FL={appliedSpeed[0]:F3}, RL={appliedSpeed[1]:F3}, RR={appliedSpeed[2]:F3}, FR={appliedSpeed[3]:F3}");
            //Debug.Log($"[appliedSteerDeg] FL={appliedSteerDeg[0]:F1}°, RL={appliedSteerDeg[1]:F1}°, RR={appliedSteerDeg[2]:F1}°, FR={appliedSteerDeg[3]:F1}°");
        }
    }

    /// <summary>
    /// PID控制器应用：将运动学解算结果（目标轮速、转向角）转换为实际控制指令
    /// 分两部分：1) 轮速PID控制电机扭矩  2) 转向角速率限制跟随
    /// </summary>
    void ApplyPIDControl()
    {
        float dt = Time.fixedDeltaTime;

        // ========== 速度反转检测 ==========
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

        // 方向反转时重置所有轮的 PID 积分器（避免积分饱和导致响应迟缓）
        if (directionChanged)
        {
            for (int j = 0; j < 4; j++)
            {
                speedPIDs[j].ResetIntegrator();
            }
            if (enableDebugLog)
            {
                Debug.Log("[ApplyPIDControl] Direction reversed! Reset all PID integrators.");
            }
        }

        // ========== 逐轮控制循环 ==========
        for (int j = 0; j < 4; j++)
        {
            WheelCollider wc = (wheelColliders != null && j < wheelColliders.Length) ? wheelColliders[j] : null;
            float wheelRadius = (wc != null) ? Mathf.Max(1e-4f, wc.radius) : 0.05f;
            
            // 获取当前轮速（从WheelCollider的RPM转换为线速度 m/s）
            // 公式：v = (RPM / 60) * 2π * r
            float current_v = 0f;
            if (wc != null) current_v = wc.rpm / 60f * 2f * Mathf.PI * wheelRadius;

            float desired_v = appliedSpeed[j];           // 目标轮速 (m/s)
            float desiredSteerDeg = appliedSteerDeg[j];  // 目标转向角 (度)

            // ========== 转向角PID跟随控制 ==========
            // 计算转向角误差（使用DeltaAngle确保最短路径，±90°范围内）
            float steerErrorDeg = Mathf.DeltaAngle(steerCmdDeg[j], desiredSteerDeg);

            float steerRateCmdDeg = 0f;
            if (Mathf.Abs(steerErrorDeg) < steerDeadbandDeg)
            {
                // 误差在死区内：停止调整
                steerPIDs[j].ResetIntegrator();
                steerRateCmdDeg = 0f;
            }
            else
            {
                // 误差超出死区：PID输出转向角速率（度/秒）
                steerRateCmdDeg = steerPIDs[j].Update(steerErrorDeg, dt);
            }

            // 更新转向角命令（向目标角度移动，速率由PID输出限制）
            // MoveTowardsAngle会自动选择最短路径（处理±180°环绕）
            steerCmdDeg[j] = Mathf.MoveTowardsAngle(steerCmdDeg[j], desiredSteerDeg, Mathf.Abs(steerRateCmdDeg) * dt);
            
            // 限制转向角在±90°范围内（与运动学解算的角度折叠保持一致）
            steerCmdDeg[j] = Mathf.Clamp(steerCmdDeg[j], -90f, 90f);
            
            // 调试：检查转向角跟随
            if (enableDebugLog && Time.frameCount % 30 == 0)
            {
                Debug.Log($"[Wheel {j}] Target={desiredSteerDeg:F1}°, Current={steerCmdDeg[j]:F1}°, Error={steerErrorDeg:F1}°, Rate={steerRateCmdDeg:F1}°/s");
            }
            
            // 计算速度误差
            float speedError = desired_v - current_v;

            // ========== 轮速PID控制 ==========
            // 根据误差大小采用不同控制策略
            if (Mathf.Abs(desired_v) < speedDeadband)
            {
                // 情况1：目标速度本身接近零 → 完全停止
                speedPIDs[j].ResetIntegrator();
                if (wc != null)
                {
                    wc.motorTorque = 0f;
                    // 自适应制动：速度越大，制动力越大
                    float autoBrake = Mathf.Clamp(brakeGain * Mathf.Abs(current_v), 0f, brakeTorqueHigh);
                    wc.brakeTorque = autoBrake;
                }
            }
            else if (Mathf.Abs(speedError) < speedDeadband)
            {
                // 情况2：误差在死区内 → 使用低增益PID维持速度（抵消摩擦力）
                // 不重置积分器，使用降低的P增益来平滑维持
                float maintainTorque = speedError * (speed_Kp * 0.3f);  // 使用30%的P增益
                maintainTorque = Mathf.Clamp(maintainTorque, -maxMotorTorque * 0.2f, maxMotorTorque * 0.2f);  // 限制在20%扭矩范围
                
                if (wc != null)
                {
                    wc.brakeTorque = 0f;
                    wc.motorTorque = maintainTorque;  // 施加维持扭矩
                }
            }
            else
            {
                // 情况3：误差超出死区 → 正常 PID 控制
                // PID输出为电机扭矩 (Nm)
                float torqueCmd = speedPIDs[j].Update(speedError, dt);
                torqueCmd = Mathf.Clamp(torqueCmd, -maxMotorTorque, maxMotorTorque);
                if (wc != null)
                {
                    wc.brakeTorque = 0f;
                    wc.motorTorque = torqueCmd;
                }
            }

            // 应用转向角到WheelCollider
            if (wc != null)
            {
                wc.steerAngle = steerCmdDeg[j];
            }

            // 记录状态供外部读取
            steerAngles[j] = steerCmdDeg[j] * Mathf.Deg2Rad;
            wheelSpeeds[j] = current_v;
        }

        // 保存当前速度命令用于下一帧比较
        for (int j = 0; j < 4; j++)
        {
            prevWheelSpeedCmd[j] = appliedSpeed[j];
        }

        // 调试打印：实际轮速和转向角
        if (enableDebugLog && Time.frameCount % 30 == 0)
        {
            Debug.Log($"[wheelSpeeds] FL={wheelSpeeds[0]:F3}, RL={wheelSpeeds[1]:F3}, RR={wheelSpeeds[2]:F3}, FR={wheelSpeeds[3]:F3}");
            Debug.Log($"[steerCmdDeg] FL={steerCmdDeg[0]:F1}°, RL={steerCmdDeg[1]:F1}°, RR={steerCmdDeg[2]:F1}°, FR={steerCmdDeg[3]:F1}°");
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
            
            // 获取位置（物理位置是准确的）
            Vector3 pos; Quaternion rot;
            wc.GetWorldPose(out pos, out rot);
            mesh.position = pos;

            if (forceWheelZto90)
            {
                // 椭圆轮专用：手动构建旋转，只使用转向角，忽略物理滚动
                // 1. 获取车体的世界旋转
                Quaternion vehicleRotation = transform.rotation;
                
                // 2. 获取轮子的转向角（相对于车体）
                float steerAngleDeg = wc.steerAngle;
                
                // 3. 构建旋转：车体旋转 + 转向（绕Y轴）+ 固定滚动90°（绕局部Z轴）
                mesh.rotation = vehicleRotation * Quaternion.Euler(0, steerAngleDeg, 90f);
            }
            else
            {
                // 使用 WheelCollider 的完整物理旋转
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