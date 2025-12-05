using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class RealCarAgent : Agent
{
    [Header("Refs")]
    public MagneticTape tape;             // 磁条引用
    public Transform[] sensors = new Transform[6]; // 磁场传感器
    public Rigidbody rb;                  // 车体刚体
    public Transform[] wheels = new Transform[4];  // 轮子的Transform（用来显示）

    [Header("Control")]
    public float maxSpeed = 0.5f;           // 最大速度
    public float maxSteerSpeed = 30f;       // 最大转向角速度
    public float maxWheelSpeed = 4.0f;      // 最大电机速度

    [Header("Normalization")]
    public float maxField = 0.02f;          // 最大磁场强度

    [Header("Reward Weights")]
    public float w_track = 1.0f;            // 磁条追踪奖励
    public float w_forward = 0.4f;          // 前进奖励
    public float w_heading = 0.2f;          // 朝向奖励

    [Header("Episode Limits")]
    public float maxEpisodeTime = 20f;      // 每回合最大时间
    private float episodeTimer;             // 回合计时器

    [Header("Start Position")]
    public Vector3 startPos = new Vector3(1f, 0.25f, -1.233f);  // 起始位置
    public Quaternion startRot = Quaternion.Euler(0f, 0f, 0f);   // 起始旋转

    private Vector3 lastForward;

    // 车辆参数
    private float L = 0.76f;  // 轴距
    private float W = 0.47f;  // 轮距

    public override void Initialize()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        lastForward = transform.forward;
    }

    public override void OnEpisodeBegin()
    {
        // 重置车辆状态
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // 设置起始位置和旋转
        transform.position = startPos;
        transform.rotation = startRot;

        lastForward = transform.forward;
        episodeTimer = 0f;

        // 确保轮子初始化时角度正确
        for (int i = 0; i < 4; i++)
        {
            wheels[i].localRotation = Quaternion.Euler(0f, 90f, 0f);  // 设置轮子z轴为90°
        }
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
        // actions[0] = 前向速度比例 (0~1)
        // actions[1] = 横向速度比例 (-1~1)
        // actions[2] = 角速度比例 (-1~1)

        // 根据动作计算速度
        float vx = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f) * maxSpeed;  // 前向速度
        float vy = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f) * maxSpeed;  // 横向速度
        float omega = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f) * maxSteerSpeed;  // 角速度

        // 死区处理：如果速度太小，就不进行任何控制
        if (Mathf.Abs(vx) < 0.1f && Mathf.Abs(vy) < 0.1f && Mathf.Abs(omega) < 0.1f)
        {
            rb.linearVelocity = Vector3.zero;  // 重置车体速度
            rb.angularVelocity = Vector3.zero;  // 重置角速度
            return;
        }

        // 轮子位置顺序：前左、后左、后右、前右
        Vector2[] wheelPos = new Vector2[4] {
            new Vector2(L / 2, W / 2),  // 前左
            new Vector2(-L / 2, W / 2), // 后左
            new Vector2(-L / 2, -W / 2),// 后右
            new Vector2(L / 2, -W / 2)  // 前右
        };

        // 计算每个轮子的速度向量
        float[] wheelSpeed = new float[4];
        float[] steerAngle = new float[4];
        for (int i = 0; i < 4; i++)
        {
            float vx_rot = -omega * wheelPos[i].y;  // 旋转速度分量
            float vy_rot = omega * wheelPos[i].x;   // 旋转速度分量
            float vx_total = vx + vx_rot;           // 总速度
            float vy_total = vy + vy_rot;           // 总速度
            steerAngle[i] = Mathf.Atan2(vy_total, vx_total);  // 计算转向角
            wheelSpeed[i] = Mathf.Sqrt(vx_total * vx_total + vy_total * vy_total);  // 计算速度大小
        }

        // 调整转向角度范围 [-π/2, π/2]
        for (int i = 0; i < 4; i++)
        {
            if (steerAngle[i] > Mathf.PI / 2)
            {
                steerAngle[i] -= Mathf.PI;
                wheelSpeed[i] = -wheelSpeed[i];
            }
            else if (steerAngle[i] < -Mathf.PI / 2)
            {
                steerAngle[i] += Mathf.PI;
                wheelSpeed[i] = -wheelSpeed[i];
            }

            // 归一化速度
            if (wheelSpeed[i] > maxWheelSpeed)
            {
                wheelSpeed[i] = maxWheelSpeed;
            }

            // 设置轮子的速度和转向角度
            wheels[i].localRotation = Quaternion.Euler(0f, 90f + Mathf.Rad2Deg * steerAngle[i], 0f);  // 控制y轴转向
        }

        // 计算奖励
        AddReward(CalculateReward());

        // 超时条件
        episodeTimer += Time.fixedDeltaTime;
        if (episodeTimer >= maxEpisodeTime)
        {
            Debug.Log("Episode Ended: Timeout reached.");
            EndEpisode();
        }

        lastForward = transform.forward;
    }

    private float CalculateReward()
    {
        // 计算奖励函数的逻辑
        float reward = 0f;

        // 奖励根据车辆前进情况
        reward += w_forward * rb.linearVelocity.magnitude * 0.01f;

        // 追踪磁条奖励
        float frontAvg = 0f;  // 前传感器平均磁场强度
        float rearAvg = 0f;   // 后传感器平均磁场强度
        for (int i = 0; i < 3; i++) frontAvg += tape.GetMagneticField(sensors[i].position).magnitude;
        for (int i = 3; i < 6; i++) rearAvg += tape.GetMagneticField(sensors[i].position).magnitude;
        frontAvg /= 3f;
        rearAvg /= 3f;
        reward += w_track * (frontAvg - rearAvg);

        // 朝向奖励
        float headingChange = Vector3.SignedAngle(lastForward, transform.forward, Vector3.up) / 180f;
        reward += w_heading * Mathf.Abs(headingChange);

        return reward;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // 手动输入的控制
        //var cont = actionsOut.ContinuousActions;
        //cont[0] = Input.GetAxis("Vertical"); // 前向速度
        //cont[1] = Input.GetAxis("Horizontal"); // 横向速度
        //cont[2] = Input.GetAxis("Mouse X"); // 角速度
    }
}
