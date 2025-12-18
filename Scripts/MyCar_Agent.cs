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

    [Header("Control limits (body frame)")]
    public float maxForwardSpeed = 0.6f;     // vx m/s
    public float maxLateralSpeed = 0.3f;     // vy m/s
    public float maxOmegaDeg = 120f;          // deg/s

    [Header("Normalization")]
    public float maxField = 8f;

    [Header("Reward Weights - 基于精确需求")]
    public float w_centerStrength = 5.0f;   // 中心传感器强度（核心目标）
    public float w_symmetry = 3.0f;         // 左右对称性（差值接近0）
    public float w_forward = 2.0f;          // 前进速度
    public float w_stability = 0.5f;        // 姿态稳定（降低权重以减少对转弯的抑制）
    
    [Header("Stability Thresholds")]
    public float symmetryThreshold = 0.3f;  // 判断左右对称的阈值（归一化，放宽以识别直角弯）
    public float centerMinThreshold = 0.3f; // 判断是否在磁条上的最低中心强度（归一化）
    
    [Header("Penalty Settings")]
    public float backwardPenaltyMultiplier = 10.0f;  // 后退惩罚倍数（完全禁止后退）
    public float minSpeedThreshold = 0.08f;          // 最低速度阈值（降低以允许转弯降速）
    public float curveSpeedBonus = 1.5f;            // 弯道时的速度奖励倍数

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
        
        sensor.AddObservation(localVel.x / Mathf.Max(0.001f, maxLateralSpeed));   // 7: 横向速度 vx
        sensor.AddObservation(localVel.z / Mathf.Max(0.001f, maxForwardSpeed));   // 8: 前进速度 vz
        
        float maxOmegaRad = maxOmegaDeg * Mathf.Deg2Rad;
        sensor.AddObservation(Mathf.Clamp(angularVel / maxOmegaRad, -1f, 1f));    // 9: 角速度 omega
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

        // 读取传感器数据（只读取一次）
        float[] sensorValues = new float[6];
        float frontMax = 0f, rearMax = 0f;
        for (int i = 0; i < sensors.Length; i++)
        {
            if (sensors[i] != null && tape != null)
            {
                Vector3 mag = tape.GetMagneticField(sensors[i].position);
                sensorValues[i] = mag.magnitude;
                if (i < 3) frontMax = Mathf.Max(frontMax, sensorValues[i]);
                else rearMax = Mathf.Max(rearMax, sensorValues[i]);
            }
        }
        
        // 终止条件：检查前后磁场是否丢失
        float lostThreshold = maxField * 0.05f;
        if (frontMax < lostThreshold || rearMax < lostThreshold)
        {
            AddReward(-1f);
            Debug.Log($"Episode Ended: magnetic signal lost. frontMax={frontMax:F4}, rearMax={rearMax:F4}, threshold={lostThreshold:F4}");
            EndEpisode();
            return;
        }

        // 计算奖励并加入（使用已读取的传感器数据）
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

        // ========== 1. 中心传感器强度（核心目标） ==========
        // 前中和后中传感器应该最强，这是完美跟踪的直接体现
        float centerAvg = (s[1] + s[4]) / 2f;  // 前中 + 后中
        float r_center = Mathf.Clamp01(centerAvg / maxField);

        // ========== 2. 左右对称性（差值接近0） ==========
        // 完美跟踪时，左右传感器值应该几乎相同
        // 整体对称性（用于奖励）：前后平均
        float leftAvg = (s[0] + s[3]) / 2f;   // 前左 + 后左
        float rightAvg = (s[2] + s[5]) / 2f;  // 前右 + 后右
        float symmetryDiff = Mathf.Abs(leftAvg - rightAvg) / maxField; // 归一化差值
        float r_symmetry = Mathf.Clamp01(1f - symmetryDiff);
        
        // 弯道判断（用于策略切换）：只看前面传感器
        // 考虑前中传感器，确保在磁条上才判断左右差异
        float frontLeftRight = Mathf.Abs(s[0] - s[2]) / maxField; // 前左 vs 前右
        bool hasFrontCenter = s[1] > maxField * 0.2f; // 前中传感器有足够信号
        // symmetryDiff = 0 → r_symmetry = 1.0（完美对称）
        // symmetryDiff = maxField → r_symmetry = 0（完全不对称）

        // ========== 3. 姿态稳定性（直线时应该稳定，转弯时允许调整） ==========
        // 区分三种状态：直线、弯道、脱轨
        // - 直线：左右对称 且 中心强度高
        // - 弯道：左右不对称 且 中心强度高（重要：避免误判脱轨为弯道）
        // - 脱轨：中心强度低（无论对称性如何）
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        float lateralSpeed = Mathf.Abs(localVel.x);
        float angularSpeed = Mathf.Abs(rb.angularVelocity.y);
        
        bool isFrontSymmetric = frontLeftRight < symmetryThreshold; // 前传感器是否对称
        bool isOnTrack = r_center > centerMinThreshold; // 中心强度足够 → 在磁条上
        bool isCurve = !isFrontSymmetric && isOnTrack && hasFrontCenter;  // 前面不对称 且 前中有信号 且 在磁条上 → 弯道
        
        float r_stability;
        if (isCurve)
        {
            // 弯道（左右不对称 且 在磁条上）：允许调整，不惩罚
            r_stability = 1f;
        }
        else if (isOnTrack)
        {
            // 直线（在磁条上 且 左右对称）：应该保持姿态稳定
            float lateralPenalty = Mathf.Clamp01(lateralSpeed / maxLateralSpeed);
            float angularPenalty = Mathf.Clamp01(angularSpeed / (maxOmegaDeg * Mathf.Deg2Rad));
            r_stability = 1f - 0.5f * (lateralPenalty + angularPenalty);
        }
        else
        {
            // 脱轨（中心强度低）：严重惩罚
            float lateralPenalty = Mathf.Clamp01(lateralSpeed / maxLateralSpeed);
            float angularPenalty = Mathf.Clamp01(angularSpeed / (maxOmegaDeg * Mathf.Deg2Rad));
            r_stability = 0.2f - 0.8f * (lateralPenalty + angularPenalty); // 最多0.2，最少-0.6
        }

        // ========== 4. 前进速度：只奖励前进，严厉惩罚后退，弯道时鼓励前进 ==========
        float forwardSpeed = localVel.z;
        
        float r_forward;
        if (forwardSpeed < -0.05f)
        {
            // 后退：严厉惩罚（完全禁止后退）
            r_forward = (forwardSpeed / maxForwardSpeed) * backwardPenaltyMultiplier;
        }
        else if (forwardSpeed > minSpeedThreshold)
        {
            // 正常前进：给予奖励
            float baseReward = Mathf.Clamp01(forwardSpeed / maxForwardSpeed);
            // 弯道时给予额外奖励，鼓励转弯
            r_forward = isCurve ? baseReward * curveSpeedBonus : baseReward;
        }
        else
        {
            // 速度过低：在弯道时不惩罚（转弯需要降速），其他情况轻微惩罚
            r_forward = isCurve ? 0f : -0.15f;
        }

        // ========== 组合奖励 ==========
        float reward = 
            w_centerStrength * r_center +    // 5.0 - 中心传感器强度（核心）
            w_symmetry * r_symmetry +        // 3.0 - 左右对称性（差值≈0）
            w_forward * r_forward +          // 2.0 - 前进速度（弯道时×1.5）
            w_stability * r_stability;       // 0.5 - 姿态稳定（降低以允许转弯）

        // 总权重为10.5，归一化并限制范围（后退惩罚可能导致超出范围）
        return Mathf.Clamp(reward / 10.5f, -2f, 1f);
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
        GUILayout.Label($"Vx: {myCarMotion.vx_input:F3} m/s | Vy: {myCarMotion.vy_input:F3} m/s | Omega: {myCarMotion.omega_input:F3} rad/s", GUILayout.Width(680));

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