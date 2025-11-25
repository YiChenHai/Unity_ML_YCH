using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class MagneticCarAgent : Agent
{
    [Header("Refs")]
    public MagneticTape tape;
    public Transform[] sensors = new Transform[6];
    public Rigidbody rb;

    [Header("Control")]
    public float maxSpeed = 0.5f;        // 最大前进速度
    public float maxSteerSpeed = 90f;    // 最大转向速度（度/秒）

    [Header("Normalization")]
    public float maxField = 0.02f;       // 磁场归一化最大值

    [Header("Reward Weights")]
    public float w_track = 1.0f;         // 磁条追踪奖励权重
    public float w_forward = 0.4f;       // 前进奖励权重
    public float w_heading = 0.2f;       // 车头朝向奖励权重

    [Header("Episode Limits")]
    public float maxEpisodeTime = 20f;
    private float episodeTimer;

    [Header("Start Position")]
    public Vector3 startPos = new Vector3(1f, 0.25f, -1.233f);
    public Quaternion startRot = Quaternion.Euler(0f, 0f, 0f);

    private Vector3 lastForward;

    public override void Initialize()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        lastForward = transform.forward;
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // 固定起始位置
        transform.position = startPos;
        transform.rotation = startRot;

        lastForward = transform.forward;
        episodeTimer = 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 获取每个传感器的磁场矢量
        Vector3[] normFields = new Vector3[sensors.Length];
        for (int i = 0; i < sensors.Length; i++)
        {
            Vector3 magVector = tape.GetMagneticField(sensors[i].position);  // 获取磁场矢量
            normFields[i] = magVector.normalized;  // 对磁场进行归一化
            sensor.AddObservation(normFields[i]); // 磁场矢量观测
        }

        // 前后平均差
        float frontAvg = (normFields[0].magnitude + normFields[1].magnitude + normFields[2].magnitude) / 3f;
        float rearAvg = (normFields[3].magnitude + normFields[4].magnitude + normFields[5].magnitude) / 3f;
        sensor.AddObservation(frontAvg - rearAvg); // 前后差

        // 左右对称差
        sensor.AddObservation(normFields[0].magnitude - normFields[2].magnitude);
        sensor.AddObservation(normFields[3].magnitude - normFields[5].magnitude);

        // 车体局部速度 XY 分量归一化
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        sensor.AddObservation(localVel.x / maxSpeed); // 横向速度
        sensor.AddObservation(localVel.z / maxSpeed); // 前向速度

        // 车头朝向变化
        float headingChange = Vector3.SignedAngle(lastForward, transform.forward, Vector3.up) / 180f;
        sensor.AddObservation(Mathf.Clamp(headingChange, -1f, 1f));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 连续动作：
        // actions[0] = 横向速度比例 (-1~1)
        // actions[1] = 前向速度比例 (0~1)
        // actions[2] = 转向角速度比例 (-1~1)
        float moveX = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float moveZ = Mathf.Clamp(actions.ContinuousActions[1], 0f, 1f);
        float steer  = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        // ---- 计算归一化磁场 ----
        float[] normFields = new float[sensors.Length];
        float maxFieldRead_Front = 0f;
        float maxFieldRead_Rear  = 0f;
        Vector3 totalFieldFront = Vector3.zero;
        Vector3 totalFieldRear  = Vector3.zero;
        for (int i = 0; i < sensors.Length / 2; i++)
        {
            Vector3 mag = tape.GetMagneticField(sensors[i].position);
            normFields[i] = Mathf.Clamp01(mag.magnitude / Mathf.Max(maxField, 1e-9f));
            totalFieldFront += mag;  // 累加前方的磁场矢量
            if (normFields[i] > maxFieldRead_Front) maxFieldRead_Front = normFields[i];
        }
        for (int i = 3; i < sensors.Length / 2 + 3; i++)
        {
            Vector3 mag = tape.GetMagneticField(sensors[i].position);
            normFields[i] = Mathf.Clamp01(mag.magnitude / Mathf.Max(maxField, 1e-9f));
            totalFieldRear += mag;  // 累加后方的磁场矢量
            if (normFields[i] > maxFieldRead_Rear) maxFieldRead_Rear = normFields[i];
        }

        // ---- 终止条件：失去磁信号 ----
        if (maxFieldRead_Front < 0.008f || maxFieldRead_Rear < 0.008f)
        {
            Debug.Log("Episode Ended: Lost magnetic signal.");
            AddReward(-1f);
            EndEpisode();
            return;
        }

        // raw actions in [-1, 1]
        float ax = actions.ContinuousActions[0]; // x方向速度归一化输入
        float az = actions.ContinuousActions[1]; // z方向速度归一化输入
        float aw = actions.ContinuousActions[2]; // 自旋归一化输入

        // 求合成速度大小
        float norm = Mathf.Abs(ax) + Mathf.Abs(az) + Mathf.Abs(aw);

        // 停止检测
        if (norm < 0.1)
        {
            Debug.Log("[终止] 车辆运动合成速度低于阈值 → Episode End");
            AddReward(-1f);
            EndEpisode();
            return;
        }

        // ---- 车辆运动 ----
        Vector3 localVel = new Vector3(moveX * maxSpeed, 0f, moveZ * maxSpeed);
        rb.linearVelocity = transform.TransformDirection(localVel);

        rb.MoveRotation(rb.rotation * Quaternion.Euler(0f, steer * maxSteerSpeed * Time.fixedDeltaTime, 0f));

        // ---- 奖励函数 ----
        float frontAvg = (normFields[0] + normFields[1] + normFields[2]) / 3f;
        float rearAvg  = (normFields[3] + normFields[4] + normFields[5]) / 3f;
        float sideDiffFront = normFields[0] - normFields[2];
        float sideDiffRear  = normFields[3] - normFields[5];

        // 磁条追踪奖励：左右平衡 + 居中
        float r_track = Mathf.Exp(-(sideDiffFront * sideDiffFront + sideDiffRear * sideDiffRear) * 10f) * ((frontAvg + rearAvg) / 2f);

        // 前进奖励：沿轨迹方向前进
        float r_forward = Mathf.Clamp01(localVel.z / maxSpeed) * ((frontAvg + rearAvg) / 2f);

        // 朝向奖励：车头朝前
        float headingError = Vector3.Angle(transform.forward, rb.linearVelocity.normalized) / 180f;
        float r_heading = 1f - Mathf.Clamp01(headingError);

        float reward = w_track * r_track + w_forward * r_forward + w_heading * r_heading;
        AddReward(reward * Time.fixedDeltaTime * 10f);

        // ---- 超时检测 ----
        episodeTimer += Time.fixedDeltaTime;
        if (episodeTimer >= maxEpisodeTime)
        {
            Debug.Log("Episode Ended: Timeout reached.");
            EndEpisode();
            return;
        }

        lastForward = transform.forward;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        cont[0] = Input.GetAxis("Horizontal"); // 横向速度
        cont[1] = Input.GetKey(KeyCode.UpArrow) ? 1f : 0f; // 前向速度
        cont[2] = Input.GetAxis("Horizontal"); // 转向角速度
    }
}
