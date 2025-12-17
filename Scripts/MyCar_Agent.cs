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
    public float maxOmegaDeg = 60f;          // deg/s

    [Header("Normalization")]
    public float maxField = 0.02f;

    [Header("Reward Weights")]
    public float w_align = 0.5f;         // 对齐磁场方向权重（降低，避免原地对齐）
    public float w_forwardField = 2.0f;  // 沿磁场方向前进权重（提高，强制前进）
    public float w_track = 1.0f;         // 磁强差奖励权重（前后传感器）
    public float w_stability = 0.5f;     // 抑制角速度/抖动权重
    public float w_bodyAlign = 0.6f;     // 车身姿态对齐权重（前后磁场一致性）
    public float w_heading = 0.2f;       // 朝向变化惩罚权重
    public float minSpeedReward = 0.2f;  // 最低速度要求（m/s），低于此速度会被惩罚

    [Header("Episode")]
    public float maxEpisodeTime = 20f;
    private float episodeTimer = 0f;

    [Header("Start pose")]
    public Vector3 startPos = new Vector3(1f, 0.25f, -1.233f);
    public Quaternion startRot = Quaternion.Euler(0f, 0f, 0f);

    private Vector3 lastForward;

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

        // 1) 用前3个传感器计算磁场方向（导航用）
        Vector3 frontField = Vector3.zero;
        for (int i = 0; i < 3; i++)
        {
            Vector3 mag = tape.GetMagneticField(sensors[i].position);
            frontField += mag;
        }
        frontField /= 3f;
        Vector3 fieldDir = new Vector3(frontField.x, 0f, frontField.z);
        if (fieldDir.sqrMagnitude < 1e-8f) return 0f;
        fieldDir.Normalize();

        // 2) 车头与磁场方向对齐奖励（只有在移动时才给对齐奖励）
        float align = Vector3.Dot(transform.forward, fieldDir); // [-1,1]
        float currentSpeed = rb.linearVelocity.magnitude;
        // 对齐奖励乘以速度系数，静止时不给对齐奖励
        float speedFactor = Mathf.Clamp01(currentSpeed / Mathf.Max(0.001f, minSpeedReward));
        float r_align = Mathf.Max(0f, align) * speedFactor; // [0,1]

        // 3) 沿磁场方向的前向速度奖励（允许负值，反向移动会被惩罚）
        float velAlong = Vector3.Dot(rb.linearVelocity, fieldDir); // m/s
        float r_forwardField = velAlong / Mathf.Max(0.001f, maxForwardSpeed); // 可为负值 [-1,1]
        
        // 额外：速度过低惩罚（鼓励保持一定速度）
        float r_minSpeed = 0f;
        if (currentSpeed < minSpeedReward)
        {
            r_minSpeed = -0.5f * (1f - currentSpeed / minSpeedReward); // [-0.5, 0]
        }

        // 4) 磁强差（前后）归一化，鼓励车头靠近磁源
        float frontAvg = 0f, rearAvg = 0f;
        for (int i = 0; i < sensors.Length; i++)
        {
            Vector3 mag = tape.GetMagneticField(sensors[i].position);
            if (i < 3) frontAvg += mag.magnitude;
            else rearAvg += mag.magnitude;
        }
        frontAvg /= 3f; rearAvg /= 3f;
        float r_track_norm = Mathf.Clamp01((frontAvg - rearAvg) / Mathf.Max(1e-6f, maxField));

        // 5) 稳定性：惩罚角速度过大并考虑朝向突变（均为归一化）
        float yawRate = rb.angularVelocity.y; // rad/s
        float maxOmegaRad = maxOmegaDeg * Mathf.Deg2Rad;
        float r_stability_rate = 1f - Mathf.Clamp01(Mathf.Abs(yawRate) / Mathf.Max(1e-6f, maxOmegaRad));

        float headingChangeDeg = Mathf.Abs(Vector3.SignedAngle(lastForward, transform.forward, Vector3.up));
        float r_stability_heading = 1f - Mathf.Clamp01(headingChangeDeg / 60f); // 60deg门限

        float r_stability = Mathf.Clamp01((r_stability_rate + r_stability_heading) * 0.5f);

        // 6) 车身姿态对齐（用后3个传感器校验）
        Vector3 rearField = Vector3.zero;
        for (int i = 3; i < 6; i++)
        {
            Vector3 mag = tape.GetMagneticField(sensors[i].position);
            rearField += mag;
        }
        rearField /= 3f;
        Vector3 rearFieldDir = new Vector3(rearField.x, 0f, rearField.z);
        
        // 车身方向应与前后磁场方向一致（如果后传感器也能检测到有效磁场）
        float r_bodyAlign = 0f;
        if (rearFieldDir.sqrMagnitude > 1e-8f)
        {
            rearFieldDir.Normalize();
            // 前后磁场方向应该一致，说明车身沿磁带
            float frontRearConsistency = Vector3.Dot(fieldDir, rearFieldDir); // [-1,1]
            r_bodyAlign = Mathf.Clamp01(frontRearConsistency); // [0,1]
        }

        // 7) 最终组合
        float reward = 
            w_align * r_align +
            w_forwardField * r_forwardField +
            w_track * r_track_norm +
            w_stability * r_stability +
            w_bodyAlign * r_bodyAlign +
            w_heading * (1f - Mathf.Clamp01(headingChangeDeg / 180f)) +
            r_minSpeed; // 低速惩罚

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