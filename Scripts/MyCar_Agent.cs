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
    public float maxForwardSpeed = 0.6f;     // vz (前进速度) m/s
    public float maxLateralSpeed = 0.3f;     // vx (横向速度) m/s
    public float maxOmegaDeg = 120f;          // omega (自转角速度) deg/s

    [Header("Normalization")]
    public float maxField = 8f;              // 磁场最大值

    [Header("Reward Weights - 极简设计")]
    public float w_alignment = 1.0f;        // 对齐（对称性）
    public float w_forward = 2.0f;          // 前进速度（提高权重，鼓励冒险前进）
    
    [Header("Penalty Settings")]
    public float backwardPenaltyMultiplier = 10.0f;  // 后退惩罚倍数（严格禁止倒车）
    public float translationThreshold = 0.2f;        // 平移运动阈值（加权后的平移强度）
    public float lateralWeight = 0.3f;               // 横向速度在平移判定中的权重（降低以防抖动exploit）
    public float rotationThreshold = 0.3f;           // 转向运动阈值（平移不足时，转向可补偿）
    public float minForwardForRotation = 0.08f;      // 旋转补偿的最低前进速度（防止原地划桨）

    [Header("Episode")]
    public float maxEpisodeTime = 20f;
    private float episodeTimer = 0f;

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

        // 清除 myCarMotion 的输入
        if (myCarMotion != null) myCarMotion.SetControl(0f, 0f, 0f);

        episodeTimer = 0f;
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

        // 7-9: 当前运动状态（车身坐标系）- 这是AI做决策的关键反馈
        Vector3 localVel = transform.InverseTransformDirection(rb != null ? rb.linearVelocity : Vector3.zero);
        float angularVel = rb != null ? rb.angularVelocity.y : 0f;
        
        sensor.AddObservation(localVel.x / Mathf.Max(0.001f, maxLateralSpeed));   // 7: 横向速度 (Unity X轴)
        sensor.AddObservation(localVel.z / Mathf.Max(0.001f, maxForwardSpeed));   // 8: 前进速度 (Unity Z轴)
        
        float maxOmegaRad = maxOmegaDeg * Mathf.Deg2Rad;
        sensor.AddObservation(Mathf.Clamp(angularVel / maxOmegaRad, -1f, 1f));    // 9: 角速度 omega
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 连续动作：0=vz比例(前进), 1=vx比例(横向), 2=omega比例(自转)
        float a_vz = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float a_vx = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float a_w  = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        // 映射到车身坐标系的真实控制量（Unity标准：Z=前进，X=横向）
        float vz = a_vz * maxForwardSpeed;  // 前进速度 (Unity Z轴)
        float vx = a_vx * maxLateralSpeed;  // 横向速度 (Unity X轴)
        float omega = a_w * maxOmegaDeg * Mathf.Deg2Rad; // 自转角速度 rad/s

        // 下发给 MyCar_Motion 去控制车辆运动
        if (myCarMotion != null) myCarMotion.SetControl(vz, vx, omega);

        // 读取传感器数据（只读取一次）
        float[] sensorValues = new float[6];
        for (int i = 0; i < sensors.Length; i++)
        {
            if (sensors[i] != null && tape != null)
            {
                Vector3 mag = tape.GetMagneticField(sensors[i].position);
                sensorValues[i] = mag.magnitude;
            }
        }
        
        // 终止条件：前中或后中丢失信号就终止（防止横移，允许弯道转向）
        float lostThreshold = maxField * 0.05f;
        float frontCenter = sensorValues[1];  // 前中
        float rearCenter = sensorValues[4];   // 后中
        
        if (frontCenter < lostThreshold || rearCenter < lostThreshold)
        {
            AddReward(-1f);
            Debug.Log($"Episode Ended: center sensor lost. frontCenter={frontCenter:F4}, rearCenter={rearCenter:F4}, threshold={lostThreshold:F4}");
            EndEpisode();
            return;
        }

        // 计算奖励并加入
        float reward = CalculateReward(sensorValues);
        AddReward(reward * Time.fixedDeltaTime);

        episodeTimer += Time.fixedDeltaTime;
        if (episodeTimer >= maxEpisodeTime)
        {
            Debug.Log($"Episode Ended: timeout. episodeTimer={episodeTimer:F2}s, maxEpisodeTime={maxEpisodeTime:F2}s");
            EndEpisode();
        } 
    }

    float CalculateReward(float[] s)
    {
        if (rb == null || s == null || s.Length < 6) return 0f;

        // ========== 1. 对齐（前后都对称 + 中心强度） ==========
        // 分别检查前排和后排的对称性
        float frontSymmetry = Mathf.Clamp01(1f - Mathf.Abs(s[0] - s[2]) / maxField);  // 前左 vs 前右
        float rearSymmetry = Mathf.Clamp01(1f - Mathf.Abs(s[3] - s[5]) / maxField);   // 后左 vs 后右
        
        // 只有前后都对称时才给高分（取最小值）
        float symmetryScore = Mathf.Min(frontSymmetry, rearSymmetry);
        
        // 中心传感器强度（独立目标）
        float centerAvg = (s[1] + s[4]) / 2f; // 前中 + 后中
        float centerStrength = Mathf.Clamp01(centerAvg / maxField);
        
        // 对齐 = 对称性 + 中心强度（两个独立目标，不相乘）
        // 这样即使偏离中心，对称性仍然有奖励，鼓励车调整回来        
        float r_alignment = (symmetryScore + centerStrength) / 2f;

        // ========== 2. 运动奖励（禁止原地对齐、禁止后退） ==========
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        float forwardSpeed = localVel.z;  // 前进方向
        float lateralSpeed = localVel.x;  // 横向
        float angularSpeed = rb.angularVelocity.y; // 自转（世界坐标系）
        
        // 分别计算平移运动和旋转运动（归一化后统一量纲）
        float vx_normalized = Mathf.Abs(lateralSpeed) / maxLateralSpeed;    // [0, 1]
        float vz_normalized = Mathf.Abs(forwardSpeed) / maxForwardSpeed;    // [0, 1]
        float omega_normalized = Mathf.Abs(angularSpeed * Mathf.Rad2Deg) / maxOmegaDeg;  // [0, 1]
        
        // 平移强度（加权：前进优先，横向次要，防止抖动exploit）
        float translationMagnitude = vz_normalized + vx_normalized * lateralWeight;
        // 旋转强度（辅助调整姿态）
        float rotationMagnitude = omega_normalized;
        
        float r_forward;
        // 动态阈值（相对于maxForwardSpeed）
        float backwardThreshold = -0.05f * (maxForwardSpeed / 0.6f);  // 按比例缩放
        float minForwardScaled = minForwardForRotation * (maxForwardSpeed / 0.6f);  // 按比例缩放
        
        if (forwardSpeed < backwardThreshold)
        {
            // 严格惩罚：后退（倒车）
            r_forward = (forwardSpeed / maxForwardSpeed) * backwardPenaltyMultiplier;
        }
        else if (translationMagnitude >= translationThreshold)
        {
            // 优先判断：平移强度足够 → 正常运动，奖励前进
            r_forward = Mathf.Clamp01(forwardSpeed / maxForwardSpeed);
        }
        else if (rotationMagnitude >= rotationThreshold && forwardSpeed >= minForwardScaled)
        {
            // 次级判断：平移不足但转向强度够 + 有基本前进速度 → 认定为姿态调整中
            // 防止原地划桨exploit，必须配合真实前进
            r_forward = 0f;
        }
        else
        {
            // 完全静止或原地划桨：平移不足、转向不足、或前进速度太低 → 严厉惩罚
            r_forward = -2.0f;
        }

        // ========== 组合奖励（极简） ==========
        float reward = w_alignment * r_alignment + w_forward * r_forward;

        // 归一化并限制范围（防止极端值）
        return Mathf.Clamp(reward / (w_alignment + w_forward), -2f, 1f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        cont[0] = Input.GetAxis("Vertical");   // vz (前进)
        cont[1] = Input.GetAxis("Horizontal"); // vx (横向)
        cont[2] = 0f;
        if (Input.GetKey(KeyCode.Q)) cont[2] = -1f;
        if (Input.GetKey(KeyCode.E)) cont[2] = 1f;
    }

    private static Texture2D _bgTexture; // 静态背景纹理，避免每帧创建

    void OnGUI()
    {
        // 调试：检查 myCarMotion 是否为空
        if (myCarMotion == null)
        {
            GUI.Label(new Rect(10, 10, 300, 50), "ERROR: myCarMotion is null! Please bind MyCar_Motion in Inspector.", 
                new GUIStyle(GUI.skin.label) { normal = { textColor = Color.red } });
            return;
        }

        // 绘制半透明灰度背景遮罩（只创建一次）
        if (_bgTexture == null)
        {
            _bgTexture = new Texture2D(1, 1);
            _bgTexture.SetPixel(0, 0, new Color(0.2f, 0.2f, 0.2f, 0.5f));
            _bgTexture.Apply();
        }
        GUI.DrawTexture(new Rect(10, 10, 700, 550), _bgTexture);

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
        GUILayout.Label($"Vz(前进): {myCarMotion.vz_input:F3} m/s | Vx(横向): {myCarMotion.vx_input:F3} m/s | Omega: {myCarMotion.omega_input:F3} rad/s", GUILayout.Width(680));

        GUILayout.EndArea();
    }

    private void OnDestroy()
    {
        // 清理静态资源
        if (_bgTexture != null)
        {
            Destroy(_bgTexture);
            _bgTexture = null;
        }
    }
}