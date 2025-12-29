using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class MyCarAgent : Agent
{
    [Header("Refs")]
    public MagneticTape tape;
    [Tooltip("传感器顺序: [0]=前左, [1]=前中, [2]=前右, [3]=后左, [4]=后中, [5]=后右")]
    public Transform[] sensors = new Transform[6];
    public Rigidbody rb;
    public MyCar_Motion myCarMotion;
 
    [Header("Control limits (body frame - Unity标准)")]
    public float constantForwardSpeed = 0.2f;  // vz 固定前进速度 m/s
    public float maxLateralSpeed = 0.8f;       // vx (横向速度) m/s
    public float maxOmegaDeg = 100f;           // omega (自转角速度) deg/s

    [Header("Normalization")]
    public float maxField = 8f;                // 磁场最大值

    [Header("Termination")]
    public float derailThreshold = 2f;         // 脱轨阈值（中心传感器低于此值终止）

    [Header("Episode")]
    public float maxEpisodeTime = 20f;  
    private float episodeTimer = 0f;

    [Header("Turn detection (front/rear diff)")]
    [Tooltip("进入转弯的前排左右差阈值 (归一化差，0~1)")]
    public float turnEnterThreshold = 0.4f;
    [Tooltip("退出转弯的前/后排左右差阈值 (滞回，0~1)")]
    public float turnExitThreshold = 0.2f;
    [Tooltip("后排确认弯道的阈值 (低一些以适应延迟感知)")]
    public float rearConfirmThreshold = 0.15f;
    [Tooltip("后排需要达到确认阈值的时间窗口 (秒)")]
    public float rearConfirmWindow = 0.5f;
    [Tooltip("退出转弯前需要连续保持低差值的时间 (秒)")]
    public float turnExitGraceTime = 0.4f;
    [Tooltip("左右差值的平滑时间常数(秒)，越小响应越快")]
    public float diffSmoothing = 0.1f;

    // 运行时状态
    private float frontDiffSmoothed = 0f;
    private float rearDiffSmoothed = 0f;
    private bool inTurnMode = false;
    private float rearConfirmTimer = 0f;
    private float turnExitTimer = 0f;
    
    // 公共访问器
    public bool IsInTurnMode => inTurnMode;
    public float FrontDiffSmoothed => frontDiffSmoothed;
    public float RearDiffSmoothed => rearDiffSmoothed;
    public float LastActionVx { get; private set; }
    public float LastActionOmega { get; private set; }
    
    // 动作平滑（内部使用）
    private float lastActionVx = 0f;
    private float lastActionOmega = 0f;
    
    [Header("Stability & Smoothing")]
    [Tooltip("动作平滑惩罚系数，越大越惩罚抖动")]
    public float actionSmoothingPenalty = 0.1f;
    [Tooltip("直线稳定奖励系数")]
    public float straightStabilityBonus = 0.5f;
    [Tooltip("直线模式下认为对齐的阈值（对称性）")]
    public float alignedThreshold = 0.8f; 
    [Tooltip("直线稳定的动作死区（绝对值），低于此值认为接近零")]
    public float straightDeadzone = 0.15f;
    [Tooltip("对齐时的动作幅度惩罚系数，越大越鼓励静止")]
    public float alignedActionPenalty = 0.3f;
    [Tooltip("是否启用软死区（对齐时直接抑制小动作）")]
    public bool useSoftDeadzone = true;
    [Tooltip("软死区阈值，对齐且动作小于此值时强制清零")]
    public float softDeadzoneThreshold = 0.10f;

    [Header("Start pose")]
    public Vector3 startPos = new Vector3(1f, 0.25f, -1.233f);
    public Quaternion startRot = Quaternion.Euler(0f, 0f, 0f);

    public override void Initialize()
    {
        base.Initialize();
        if (rb == null) rb = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin() 
    {
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
        transform.position = startPos;
        transform.rotation = startRot;

        // 设置固定前进速度，清除其他输入
        if (myCarMotion != null) myCarMotion.SetControl(constantForwardSpeed, 0f, 0f);

        episodeTimer = 0f;
        frontDiffSmoothed = 0f;
        rearDiffSmoothed = 0f;
        inTurnMode = false;
        rearConfirmTimer = 0f;
        turnExitTimer = 0f;
        lastActionVx = 0f;
        lastActionOmega = 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1-6: 六个传感器的归一化强度（环境感知）
        for (int i = 0; i < sensors.Length; i++)
        {
            if (sensors[i] != null && tape != null)
            {
                Vector3 mag = tape.GetMagneticField(sensors[i].position);
                sensor.AddObservation(Mathf.Clamp01(mag.magnitude / Mathf.Max(1e-9f, maxField)));
            }
            else sensor.AddObservation(0f);
        }

        // 7-9: 当前运动状态（车身坐标系）- AI决策反馈
        Vector3 localVel = transform.InverseTransformDirection(rb != null ? rb.linearVelocity : Vector3.zero);
        float angularVel = rb != null ? rb.angularVelocity.y : 0f;
        
        sensor.AddObservation(localVel.z / Mathf.Max(0.001f, constantForwardSpeed));  // 7: 前进速度 (Unity Z轴)
        sensor.AddObservation(localVel.x / Mathf.Max(0.001f, maxLateralSpeed));       // 8: 横向速度 (Unity X轴)
        
        float maxOmegaRad = maxOmegaDeg * Mathf.Deg2Rad;
        sensor.AddObservation(Mathf.Clamp(angularVel / maxOmegaRad, -1f, 1f));        // 9: 角速度 omega

        // 10-12: 转弯判定信号（使用动作阶段更新后的平滑值 + 状态标志）
        sensor.AddObservation(frontDiffSmoothed);          // 10: 前排左右差平滑值
        sensor.AddObservation(rearDiffSmoothed);           // 11: 后排左右差平滑值
        sensor.AddObservation(inTurnMode ? 1f : 0f);       // 12: 转弯模式标志
    }

    public override void OnActionReceived(ActionBuffers actions)
    { 
        // 连续动作：0=vx比例(横向), 1=omega比例(自转)
        float a_vx = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float a_w  = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        
        // 软死区：对齐时抑制小动作，强制车辆保持稳定
        if (useSoftDeadzone && !inTurnMode)
        {
            float[] sensorValuesTemp = new float[6];
            for (int i = 0; i < sensors.Length; i++)
            {
                if (sensors[i] != null && tape != null)
                {
                    sensorValuesTemp[i] = tape.GetMagneticField(sensors[i].position).magnitude;
                }
            }
            
            float frontSym = Mathf.Clamp01(1f - Mathf.Abs(sensorValuesTemp[0] - sensorValuesTemp[2]) / maxField);
            float rearSym = Mathf.Clamp01(1f - Mathf.Abs(sensorValuesTemp[3] - sensorValuesTemp[5]) / maxField);
            float currentAlignment = Mathf.Min(frontSym, rearSym);
            
            if (currentAlignment >= alignedThreshold)
            {
                if (Mathf.Abs(a_vx) < softDeadzoneThreshold) a_vx = 0f;
                if (Mathf.Abs(a_w) < softDeadzoneThreshold) a_w = 0f;
            }
        }

        // 映射到真实控制量（vz固定，只控制vx和omega）
        float vz = constantForwardSpeed;                        // 固定前进速度
        float vx = a_vx * maxLateralSpeed;                     // 横向速度
        float omega = a_w * maxOmegaDeg * Mathf.Deg2Rad;       // 自转角速度 rad/s

        // 下发给 MyCar_Motion 控制车辆
        if (myCarMotion != null) myCarMotion.SetControl(vz, vx, omega);

        // 读取传感器数据
        float[] sensorValues = new float[6];
        for (int i = 0; i < sensors.Length; i++)
        {
            if (sensors[i] != null && tape != null)
            {
                Vector3 mag = tape.GetMagneticField(sensors[i].position);
                sensorValues[i] = mag.magnitude;
            }
        }

        // 转弯判定：前排用于启动，后排用于确认/退出，带滞回和平滑
        float frontDiffNow = ComputeNormalizedDiff(sensorValues[0], sensorValues[2]);
        float rearDiffNow = ComputeNormalizedDiff(sensorValues[3], sensorValues[5]);
        UpdateTurnDetection(frontDiffNow, rearDiffNow, Time.fixedDeltaTime);
        
        // ========== 终止条件1：脱轨检测 ==========
        float frontCenter = sensorValues[1];  // 前中
        float rearCenter = sensorValues[4];   // 后中
        
        if (frontCenter < derailThreshold || rearCenter < derailThreshold)
        {
            AddReward(-5f);
            Debug.Log($"Episode Ended: derailment. frontCenter={frontCenter:F4}, rearCenter={rearCenter:F4}");
            EndEpisode();
            return;
        }

        // ========== 计算对齐奖励 ==========
        float reward = CalculateReward(sensorValues);
        AddReward(reward * Time.fixedDeltaTime);
        
        // ========== 动作平滑惩罚 ==========
        float actionChange = Mathf.Abs(a_vx - lastActionVx) + Mathf.Abs(a_w - lastActionOmega);
        AddReward(-actionSmoothingPenalty * actionChange * Time.fixedDeltaTime);
        
        // ========== 直线稳定奖励 + 对齐动作惩罚 ==========
        if (!inTurnMode)
        {
            // 计算当前对齐度
            float frontSym = Mathf.Clamp01(1f - Mathf.Abs(sensorValues[0] - sensorValues[2]) / maxField);
            float rearSym = Mathf.Clamp01(1f - Mathf.Abs(sensorValues[3] - sensorValues[5]) / maxField);
            float alignment = Mathf.Min(frontSym, rearSym);
            
            // 对齐度高时，惩罚动作幅度（鼓励静止）
            if (alignment >= alignedThreshold)
            {
                float actionMagnitude = Mathf.Abs(a_vx) + Mathf.Abs(a_w);
                AddReward(-alignedActionPenalty * actionMagnitude * Time.fixedDeltaTime);
                
                // 动作在死区内给额外稳定奖励
                bool vxInDeadzone = Mathf.Abs(a_vx) <= straightDeadzone;
                bool omegaInDeadzone = Mathf.Abs(a_w) <= straightDeadzone;
                
                if (vxInDeadzone && omegaInDeadzone)
                {
                    AddReward(straightStabilityBonus * Time.fixedDeltaTime);
                }
            }
        }
        
        lastActionVx = a_vx;
        lastActionOmega = a_w;
        LastActionVx = a_vx;
        LastActionOmega = a_w;
 
        // ========== 终止条件2：超时 ==========
        episodeTimer += Time.fixedDeltaTime;
        if (episodeTimer >= maxEpisodeTime)
        {
            Debug.Log($"Episode Ended: timeout. episodeTimer={episodeTimer:F2}s");
            EndEpisode();
        }  
    }

    float CalculateReward(float[] s)
    {
        if (s == null || s.Length < 6) return 0f;

        // ========== 对齐奖励：前后左右对称性 ==========
        // 前排对称：前左 vs 前右
        float frontSymmetry = Mathf.Clamp01(1f - Mathf.Abs(s[0] - s[2]) / maxField);
        // 后排对称：后左 vs 后右
        float rearSymmetry = Mathf.Clamp01(1f - Mathf.Abs(s[3] - s[5]) / maxField);
        
        // 只有前后都对称时才给高分（取最小值，确保整车对齐）
        float alignment = Mathf.Min(frontSymmetry, rearSymmetry);

        // ========== 前进速度因子：分段式速度奖励（转弯宽容） ==========
        Vector3 vel = rb != null ? rb.linearVelocity : Vector3.zero;
        float forwardSpeed = Vector3.Dot(vel, transform.forward);  // 实际前进速度
        
        // 直线时要求更高速度，转弯时放宽一点
        float speedThreshold = inTurnMode ? constantForwardSpeed * 0.45f : constantForwardSpeed * 0.6f;
        float speedRatio;
        
        if (forwardSpeed >= speedThreshold)
        {
            // 速度达到阈值（直线约60%，转弯约45%目标），给予全额奖励
            speedRatio = 1.0f;
        }
        else if (forwardSpeed >= 0.05f)
        {
            // 速度介于5cm/s和阈值之间，线性衰减
            speedRatio = forwardSpeed / speedThreshold;
        }
        else
        {
            // 几乎停止（<5cm/s），无奖励
            speedRatio = 0f;
        }
        
        // 最终奖励 = 对齐分数 × 前进因子
        // 转弯时只要保持≥60%目标速度，就不会损失奖励
        return alignment * speedRatio;
    }

    // 归一化左右差：|L-R| / max(|L|+|R|, eps)，范围 0~1
    float ComputeNormalizedDiff(float left, float right)
    {
        float denom = Mathf.Max(Mathf.Abs(left) + Mathf.Abs(right), 1e-4f);
        return Mathf.Clamp01(Mathf.Abs(left - right) / denom);
    }

    // 转弯模式判定：前排触发，后排确认/退出，带时间滞回与平滑
    void UpdateTurnDetection(float frontDiff, float rearDiff, float dt)
    {
        // 指数平滑：alpha 基于时间常数和 dt，避免步长变化导致响应不一致
        float alpha = 1f - Mathf.Exp(-dt / Mathf.Max(1e-4f, diffSmoothing));
        frontDiffSmoothed = Mathf.Lerp(frontDiffSmoothed, frontDiff, alpha);
        rearDiffSmoothed = Mathf.Lerp(rearDiffSmoothed, rearDiff, alpha);

        // 进入：前排超过进入阈值
        if (!inTurnMode && frontDiffSmoothed >= turnEnterThreshold)
        {
            inTurnMode = true;
            rearConfirmTimer = 0f;
            turnExitTimer = 0f;
        }

        if (inTurnMode)
        {
            // 后排确认：在窗口内累积时间，只要确认过就认为弯在持续
            if (rearDiffSmoothed >= rearConfirmThreshold)
            {
                rearConfirmTimer = Mathf.Min(rearConfirmTimer + dt, rearConfirmWindow);
            }
            else
            {
                // 若后排长时间低于阈值，计时器缓慢衰减，避免瞬时掉落就退出
                rearConfirmTimer = Mathf.Max(0f, rearConfirmTimer - dt * 0.5f);
            }

            // 退出条件：前后差都低于退出阈值，且维持一定时间
            bool frontLow = frontDiffSmoothed <= turnExitThreshold;
            bool rearLow = rearDiffSmoothed <= turnExitThreshold;

            if (frontLow && rearLow)
            {
                turnExitTimer += dt;
            }
            else
            {
                turnExitTimer = 0f;
            }

            // 防误判：如果后排一直未确认且前排显著回落，也允许退出
            bool noRearConfirm = rearConfirmTimer < 0.05f;
            bool frontBackToStraight = frontDiffSmoothed < turnEnterThreshold * 0.6f;
            if (noRearConfirm && frontBackToStraight)
            {
                turnExitTimer += dt;
            }

            // 打印当前状态和未退出原因
            if (turnExitTimer < turnExitGraceTime)
            {
                string reason = "";
                if (!(frontLow && rearLow))
                {
                    if (!frontLow) reason += $"frontDiffSmoothed={frontDiffSmoothed:F3} > turnExitThreshold={turnExitThreshold:F3}; ";
                    if (!rearLow) reason += $"rearDiffSmoothed={rearDiffSmoothed:F3} > turnExitThreshold={turnExitThreshold:F3}; ";
                }
                if (frontLow && rearLow && turnExitTimer < turnExitGraceTime)
                {
                    reason += $"turnExitTimer={turnExitTimer:F3} < turnExitGraceTime={turnExitGraceTime:F3}; ";
                }
                if (noRearConfirm && !frontBackToStraight)
                {
                    reason += $"noRearConfirm(rearConfirmTimer={rearConfirmTimer:F3})且frontDiffSmoothed未显著回落; ";
                }
                Debug.Log($"[转弯模式] 未退出，原因: {reason}");
            }

            if (turnExitTimer >= turnExitGraceTime)
            {
                Debug.Log($"[转弯模式] 满足退出条件，退出转弯模式。");
                inTurnMode = false;
                rearConfirmTimer = 0f;
                turnExitTimer = 0f;
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // 不需要手动控制
    }
}