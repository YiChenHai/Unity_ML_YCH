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
    public float maxOmegaDeg = 240f;           // omega (自转角速度) deg/s

    [Header("Normalization")]
    public float maxField = 8f;                // 磁场最大值

    [Header("Termination")]
    public float derailThreshold = 2f;         // 脱轨阈值（中心传感器低于此值终止）

    [Header("Episode")]
    public float maxEpisodeTime = 20f;
    public float speedCheckProtectionTime = 2.0f;  // 开始后保护时间（秒），期间不检测低速
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

        // 设置固定前进速度，清除其他输入
        if (myCarMotion != null) myCarMotion.SetControl(constantForwardSpeed, 0f, 0f);

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

        // 7-8: 当前运动状态（车身坐标系）- AI决策反馈
        Vector3 localVel = transform.InverseTransformDirection(rb != null ? rb.linearVelocity : Vector3.zero);
        float angularVel = rb != null ? rb.angularVelocity.y : 0f;
        
        sensor.AddObservation(localVel.x / Mathf.Max(0.001f, maxLateralSpeed));   // 7: 横向速度 (Unity X轴)
        
        float maxOmegaRad = maxOmegaDeg * Mathf.Deg2Rad;
        sensor.AddObservation(Mathf.Clamp(angularVel / maxOmegaRad, -1f, 1f));    // 8: 角速度 omega
    }

    public override void OnActionReceived(ActionBuffers actions)
    { 
        // 连续动作：0=vx比例(横向), 1=omega比例(自转)
        float a_vx = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float a_w  = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

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

        // ========== 终止条件2：前进速度过低检测（带保护时间）==========
        Vector3 vel = rb != null ? rb.linearVelocity : Vector3.zero;
        float forwardSpeed = Vector3.Dot(vel, transform.forward);  // 前进方向速度分量
        if (episodeTimer > speedCheckProtectionTime && forwardSpeed < 0.1f)
        {
            AddReward(-3f);
            Debug.Log($"Episode Ended: forward speed too low. forwardSpeed={forwardSpeed:F4} m/s (time={episodeTimer:F2}s)");
            EndEpisode();
            return;
        }

        // ========== 计算对齐奖励 ==========
        float reward = CalculateReward(sensorValues);
        AddReward(reward * Time.fixedDeltaTime);

        // ========== 终止条件3：超时 ==========
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

        // 返回对齐奖励 [0, 1]
        return alignment;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // 不需要手动控制
    }
}