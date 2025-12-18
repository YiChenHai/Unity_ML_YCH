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
    public Transform[] sensors = new Transform[6];
    public Rigidbody rb;
    public MyCar_Motion myCarMotion;

    [Header("Control limits (body frame)")]
    public float maxForwardSpeed = 0.6f;     // vx m/s
    public float maxLateralSpeed = 0.3f;     // vy m/s
    public float maxOmegaDeg = 120f;          // deg/s

    [Header("Normalization")]
    public float maxField = 8f;

    [Header("Reward Weights")]
    public float w_align = 0.5f;         // 对齐磁场方向权重（降低，避免原地对齐）
    public float w_forwardField = 2.0f;  // 沿磁场方向前进权重（提高，强制前进）
    public float w_track = 1.0f;         // 磁强差奖励权重（前后传感器）
    public float w_stability = 0.2f;     // 抑制角速度/抖动权重（降低，允许转向）
    public float w_bodyAlign = 0.6f;     // 车身姿态对齐权重（前后磁场一致性）
    public float w_heading = 0.2f;       // 朝向变化惩罚权重
    public float w_turning = 0.8f;       // 转向引导权重（新增）
    public float minSpeedReward = 0.2f;  // 最低速度要求（m/s），低于此速度会被惩罚

    [Header("Episode")]
    public float maxEpisodeTime = 20f;
    private float episodeTimer = 0f;

    [Header("Start pose")]
    public Vector3 startPos = new Vector3(1f, 0.25f, -1.233f);
    public Quaternion startRot = Quaternion.Euler(0f, 0f, 0f);

    private Vector3 lastForward;
    private float lastTotalStrength = 0f;
    private float cumulativeForwardDistance = 0f;
    private Vector3 lastPosition;

    public override void Initialize()
    {
        base.Initialize();
        if (rb == null) rb = GetComponent<Rigidbody>();
        lastForward = transform.forward;
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

        // 清除 myCarMotion 的输入
        if (myCarMotion != null) myCarMotion.SetControl(0f, 0f, 0f);

        episodeTimer = 0f;
        lastForward = transform.forward;
        lastTotalStrength = 0f;
        cumulativeForwardDistance = 0f;
        lastPosition = transform.position;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 每个传感器：使用归一化强度
        for (int i = 0; i < sensors.Length; i++)
        {
            if (sensors[i] != null && tape != null)
            {
                Vector3 mag = tape.GetMagneticField(sensors[i].position);
                sensor.AddObservation(Mathf.Clamp01(mag.magnitude / Mathf.Max(1e-9f, maxField)));
            }
            else sensor.AddObservation(0f);
        }

        // 局部速度
        Vector3 localVel = transform.InverseTransformDirection(rb != null ? rb.linearVelocity : Vector3.zero);
        sensor.AddObservation(localVel.x / Mathf.Max(0.001f, maxLateralSpeed));
        sensor.AddObservation(localVel.z / Mathf.Max(0.001f, maxForwardSpeed));

        // heading change
        float headingChange = Vector3.SignedAngle(lastForward, transform.forward, Vector3.up) / 180f;
        sensor.AddObservation(Mathf.Clamp(headingChange, -1f, 1f));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 连续动作：0=vx比例, 1=vy比例, 2=omega比例
        float a_vx = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float a_vy = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float a_w  = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        // 映射到车身坐标系的真实控制量
        float vx = a_vx * maxForwardSpeed;
        float vy = a_vy * maxLateralSpeed;
        float omega = a_w * maxOmegaDeg * Mathf.Deg2Rad; // rad/s

        // 下发给 MyCar_Motion 去控制车辆运动
        if (myCarMotion != null) myCarMotion.SetControl(vx, vy, omega);

        // 计算奖励并加入
        float reward = CalculateReward();
        AddReward(reward * Time.fixedDeltaTime);

        // 终止条件：检查前后磁场是否丢失
        float frontMax = 0f, rearMax = 0f;
        for (int i = 0; i < sensors.Length; i++)
        {
            if (sensors[i] == null || tape == null) continue;
            Vector3 mag = tape.GetMagneticField(sensors[i].position);
            float s = mag.magnitude;
            if (i < 3) frontMax = Mathf.Max(frontMax, s);
            else rearMax = Mathf.Max(rearMax, s);
        }
        if (frontMax < 0.005f || rearMax < 0.005f)
        {
            AddReward(-1f);
            Debug.Log($"Episode Ended: magnetic signal lost. frontMax={frontMax:F4}, rearMax={rearMax:F4}");
            EndEpisode();
            return;
        }

        episodeTimer += Time.fixedDeltaTime;
        if (episodeTimer >= maxEpisodeTime)
        {
            Debug.Log($"Episode Ended: timeout. episodeTimer={episodeTimer:F2}s, maxEpisodeTime={maxEpisodeTime:F2}s");
            EndEpisode();
        }

        lastForward = transform.forward;
    }

    float CalculateReward()
    {
        if (tape == null || sensors == null || sensors.Length < 6 || rb == null) return 0f;

        // 获取6个传感器的标量强度
        // 0:前左 1:前中 2:前右 3:后左 4:后中 5:后右
        float[] strength = new float[6];
        for (int i = 0; i < 6; i++)
        {
            Vector3 mag = tape.GetMagneticField(sensors[i].position);
            strength[i] = mag.magnitude;
        }

        // 计算各区域平均强度
        float frontAvg = (strength[0] + strength[1] + strength[2]) / 3f;
        float rearAvg = (strength[3] + strength[4] + strength[5]) / 3f;
        float leftAvg = (strength[0] + strength[3]) / 2f;
        float rightAvg = (strength[2] + strength[5]) / 2f;
        float centerAvg = (strength[1] + strength[4]) / 2f;
        float totalStrength = (frontAvg + rearAvg) / 2f;

        // ============ 奖励分量 ============

        // 1) 前后强度差 - 鼓励车头指向磁带方向
        float r_frontRear = Mathf.Clamp((frontAvg - rearAvg) / Mathf.Max(1e-6f, maxField), -1f, 1f);

        // 2) 左右对称性 - 鼓励车身居中
        float lateralImbalance = Mathf.Abs(leftAvg - rightAvg) / Mathf.Max(1e-6f, maxField);
        float r_centered = 1f - Mathf.Clamp01(lateralImbalance);

        // 3) 整体强度 - 保持在磁带上
        float r_onTrack = Mathf.Clamp01(totalStrength / Mathf.Max(1e-6f, maxField));

        // 4) 强度增长率 - 奖励接近磁带
        float strengthChange = totalStrength - lastTotalStrength;
        float r_approaching = Mathf.Clamp(strengthChange / Mathf.Max(1e-6f, maxField * Time.fixedDeltaTime), -1f, 1f);
        lastTotalStrength = totalStrength;

        // 5) 前向速度 - 鼓励沿车头方向移动
        float forwardSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);
        float r_forwardSpeed = Mathf.Clamp(forwardSpeed / Mathf.Max(0.001f, maxForwardSpeed), -2f, 1f);
        // 倒退惩罚加倍

        // 6) 总速度 - 基础移动鼓励
        float currentSpeed = rb.linearVelocity.magnitude;
        float r_speed = Mathf.Clamp01(currentSpeed / Mathf.Max(0.001f, maxForwardSpeed));

        // 7) 累计前进距离（沿车头方向）
        Vector3 displacement = transform.position - lastPosition;
        float forwardDisplacement = Vector3.Dot(displacement, transform.forward);
        cumulativeForwardDistance += forwardDisplacement;
        lastPosition = transform.position;
        float r_progress = forwardDisplacement / Mathf.Max(0.001f, maxForwardSpeed * Time.fixedDeltaTime);

        // 8) 稳定性 - 抑制过度抖动
        float yawRate = Mathf.Abs(rb.angularVelocity.y);
        float maxOmegaRad = maxOmegaDeg * Mathf.Deg2Rad;
        float r_stability = 1f - Mathf.Clamp01(yawRate / Mathf.Max(1e-6f, maxOmegaRad));

        // 9) 转向引导 - 根据左右传感器差异指示转向
        // 左侧强度 > 右侧 → 应该右转（正角速度）
        // 右侧强度 > 左侧 → 应该左转（负角速度）
        float lateralGradient = (rightAvg - leftAvg) / Mathf.Max(1e-6f, maxField); // [-1,1]
        float desiredOmegaSign = Mathf.Sign(lateralGradient); // 期望的旋转方向
        float actualOmegaSign = Mathf.Sign(rb.angularVelocity.y);
        
        float r_turning = 0f;
        if (Mathf.Abs(lateralGradient) > 0.1f) // 只在明显偏离时引导转向
        {
            // 如果转向方向正确，给奖励
            if (desiredOmegaSign == actualOmegaSign)
            {
                r_turning = Mathf.Abs(lateralGradient) * Mathf.Clamp01(Mathf.Abs(yawRate) / maxOmegaRad);
            }
            else
            {
                // 转向方向错误，轻微惩罚
                r_turning = -0.3f * Mathf.Abs(lateralGradient);
            }
        }

        // 10) 低速惩罚
        float r_minSpeed = 0f;
        if (currentSpeed < minSpeedReward)
        {
            r_minSpeed = -1.0f * (1f - currentSpeed / Mathf.Max(0.001f, minSpeedReward));
        }

        // ============ 权重组合 ============
        float reward = 
            w_forwardField * r_forwardSpeed +     // 2.0 前向速度最重要
            w_track * r_frontRear +                // 1.0 前后梯度
            w_turning * r_turning +                // 0.8 转向引导（新增）
            w_bodyAlign * r_centered +             // 0.6 左右居中
            w_align * r_onTrack +                  // 0.5 保持在磁带上
            0.3f * r_progress +                    // 0.3 累计前进
            w_stability * r_stability +            // 0.2 稳定性（降权）
            0.2f * r_approaching +                 // 0.2 接近磁带
            r_minSpeed;                            // 低速惩罚

        return reward;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        cont[0] = Input.GetAxis("Vertical");   // vx
        cont[1] = Input.GetAxis("Horizontal"); // vy
        cont[2] = 0f;
        if (Input.GetKey(KeyCode.Q)) cont[2] = -1f;
        if (Input.GetKey(KeyCode.E)) cont[2] = 1f;
    }

    void OnGUI()
    {
        // 调试：检查 myCarMotion 是否为空
        if (myCarMotion == null)
        {
            GUI.Label(new Rect(10, 10, 300, 50), "ERROR: myCarMotion is null! Please bind MyCar_Motion in Inspector.", 
                new GUIStyle(GUI.skin.label) { normal = { textColor = Color.red } });
            return;
        }

        // 绘制半透明灰度背景遮罩
        Texture2D backgroundTexture = new Texture2D(1, 1);
        backgroundTexture.SetPixel(0, 0, new Color(0.2f, 0.2f, 0.2f, 0.5f));
        backgroundTexture.Apply();
        GUI.DrawTexture(new Rect(10, 10, 700, 550), backgroundTexture);

        GUILayout.BeginArea(new Rect(10, 10, 700, 550));
        GUILayout.Box("Vehicle & Wheel Info", GUILayout.Width(680));

        // ========== 整车信息 ==========
        GUILayout.Label("═══ Vehicle (Body) ═══", GUILayout.Width(680));
        
        Vector3 vel = rb != null ? rb.linearVelocity : Vector3.zero;
        float speed = vel.magnitude;
        float forwardSpeed = Vector3.Dot(vel, transform.forward);
        float lateralSpeed = Vector3.Dot(vel, transform.right);
        
        Vector3 angVel = rb != null ? rb.angularVelocity : Vector3.zero;
        float yawRate = angVel.y * Mathf.Rad2Deg; // deg/s

        GUILayout.Label($"Linear Velocity: {speed:F2} m/s (Forward: {forwardSpeed:F2}, Lateral: {lateralSpeed:F2})", GUILayout.Width(680));
        GUILayout.Label($"Yaw Rate: {yawRate:F1} deg/s | Position: ({transform.position.x:F2}, {transform.position.z:F2})", GUILayout.Width(680));

        GUILayout.Space(10);

        // ========== 各轮子信息（绿色显示）==========
        GUILayout.Label("═══ Wheels ═══", GUILayout.Width(680));

        string[] wheelNames = { "FL", "RL", "RR", "FR" };
        GUIStyle greenLabelStyle = new GUIStyle(GUI.skin.label)
        {
            normal = { textColor = Color.green },
            fontSize = 12,
            fontStyle = FontStyle.Bold
        };
        
        for (int i = 0; i < 4; i++)
        {
            WheelCollider wc = (myCarMotion.wheelColliders != null && i < myCarMotion.wheelColliders.Length) 
                ? myCarMotion.wheelColliders[i] 
                : null;

            float steerDeg = (myCarMotion.steerAngles != null && i < myCarMotion.steerAngles.Length) 
                ? myCarMotion.steerAngles[i] * Mathf.Rad2Deg 
                : 0f;
            
            float wheelSpeed = (myCarMotion.wheelSpeeds != null && i < myCarMotion.wheelSpeeds.Length) 
                ? myCarMotion.wheelSpeeds[i] 
                : 0f;

            float motorTorque = wc != null ? wc.motorTorque : 0f;
            float brakeTorque = wc != null ? wc.brakeTorque : 0f;
            float wheelRpm = wc != null ? wc.rpm : 0f;

            GUILayout.Label($"{wheelNames[i]}: Speed={wheelSpeed:F2}m/s Angle={steerDeg:F1}° RPM={wheelRpm:F0} | Motor={motorTorque:F1}Nm Brake={brakeTorque:F1}Nm", greenLabelStyle, GUILayout.Width(680));
        }

        GUILayout.Space(10);

        // ========== 控制输入信息 ==========
        GUILayout.Label("═══ Control Input ═══", GUILayout.Width(680));
        GUILayout.Label($"Vx: {myCarMotion.vx_input:F3} m/s | Vy: {myCarMotion.vy_input:F3} m/s | Omega: {myCarMotion.omega_input:F3} rad/s", GUILayout.Width(680));

        GUILayout.EndArea();
    }
}