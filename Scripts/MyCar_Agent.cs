// ...existing code...
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
    public MyCar_Motion myCarMotion; // 改为新的类名并在 Inspector 里绑定 MyCar_Motion 组件

    [Header("Control limits (body frame)")]
    public float maxForwardSpeed = 0.6f;     // vx m/s
    public float maxLateralSpeed = 0.3f;     // vy m/s
    public float maxOmegaDeg = 60f;          // deg/s

    [Header("Normalization")]
    public float maxField = 0.02f;

    [Header("Reward Weights")]
    public float w_track = 1.0f;
    public float w_forward = 0.4f;
    public float w_heading = 0.2f;

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

        // 清除 myCarMotion 的输入（MyCar_Motion 内部会在 FixedUpdate 运行驱动）
        if (myCarMotion != null) myCarMotion.SetControl(0f, 0f, 0f);

        episodeTimer = 0f;
        lastForward = transform.forward;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 每个传感器：使用归一化强度 (也可以按需加入方向)
        for (int i = 0; i < sensors.Length; i++)
        {
            if (sensors[i] != null && tape != null)
            {
                Vector3 mag = tape.GetMagneticField(sensors[i].position);
                sensor.AddObservation(Mathf.Clamp01(mag.magnitude / Mathf.Max(1e-9f, maxField)));
            }
            else sensor.AddObservation(0f);
        }

        // 局部速度 (right=x, forward=z)
        Vector3 localVel = transform.InverseTransformDirection(rb != null ? rb.linearVelocity : Vector3.zero);
        sensor.AddObservation(localVel.x / Mathf.Max(0.001f, maxLateralSpeed));
        sensor.AddObservation(localVel.z / Mathf.Max(0.001f, maxForwardSpeed));

        // heading change
        float headingChange = Vector3.SignedAngle(lastForward, transform.forward, Vector3.up) / 180f;
        sensor.AddObservation(Mathf.Clamp(headingChange, -1f, 1f));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 连续动作：0=vx比例(-1..1),1=vy比例(-1..1),2=omega比例(-1..1)
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
        if (tape == null || sensors == null || sensors.Length < 6) return 0f;

        float frontAvg = 0f, rearAvg = 0f;
        for (int i = 0; i < sensors.Length; i++)
        {
            Vector3 mag = tape.GetMagneticField(sensors[i].position);
            if (i < 3) frontAvg += mag.magnitude;
            else rearAvg += mag.magnitude;
        }
        frontAvg /= 3f; rearAvg /= 3f;

        float r_track = frontAvg - rearAvg;
        float forwardSpeed = Vector3.Dot(rb != null ? rb.linearVelocity : Vector3.zero, transform.forward);
        float r_forward = Mathf.Max(0f, forwardSpeed) / Mathf.Max(0.001f, maxForwardSpeed);
        float headingChange = Mathf.Abs(Vector3.SignedAngle(lastForward, transform.forward, Vector3.up)) / 180f;
        float r_heading = 1f - Mathf.Clamp01(headingChange);

        return w_track * r_track + w_forward * r_forward + w_heading * r_heading;
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
}
// ...existing code...