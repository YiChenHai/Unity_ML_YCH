// ...existing code...
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class RealCarAgent : Agent
{
    [Header("Refs")]
    public MagneticTape tape;                     // 磁条引用（外部脚本）
    public Transform[] sensors = new Transform[6]; // 6 个磁传感器
    public Rigidbody rb;                           // 车体刚体

    [Header("Wheels (order: FL, RL, RR, FR)")]
    public WheelCollider[] wheelColliders = new WheelCollider[4]; // WheelColliders 顺序必须为 FL, RL, RR, FR
    public Transform[] wheels = new Transform[4];                // 可视轮子 Transform，顺序同上

    [Header("Control")]
    public float maxSpeed = 1.0f;          // 车体期望最大线速度 (m/s)
    public float maxSteerSpeed = 60f;      // 输入角速度最大值 (deg/s)
    public float maxWheelSpeed = 6.0f;     // 单轮线速度上限 (m/s)

    [Header("Wheel Physics")]
    public float maxMotorTorque = 200f;    // 最大电机扭矩 (N·m)
    public float torqueGain = 400f;        // P 控制增益：把速度误差转为扭矩
    public float brakeTorqueHigh = 1000f;  // 用于快速停车的高制动力
    public float brakeGain = 800f;         // 根据当前轮速施加比例制动力，避免滑行
    public float comHeightOffset = -0.12f; // 下调重心 (m)
    public float rbLinearDrag = 0.15f;     // 刚体线性阻力 (drag)
    public float rbAngularDrag = 0.05f;    // 刚体角阻力 (angularDrag)

    [Header("Normalization")]
    public float maxField = 0.02f;

    [Header("Reward Weights")]
    public float w_track = 1.0f;
    public float w_forward = 0.4f;
    public float w_heading = 0.2f;

    [Header("Episode Limits")]
    public float maxEpisodeTime = 20f;
    private float episodeTimer;

    [Header("Start Position")]
    public Vector3 startPos = new Vector3(1f, 0.25f, -1.233f);
    public Quaternion startRot = Quaternion.Euler(0f, 0f, 0f);

    private Vector3 lastForward;

    // 车辆几何参数（轴距 L，轮距 W），用于运动学求解
    private float L = 0.76f;  // 轴距 (m)
    private float W = 0.47f;  // 轮距 (m)

    public override void Initialize()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        lastForward = transform.forward;

        // 下调重心以减少被抬起的情况（可在 Inspector 微调）
        rb.centerOfMass += new Vector3(0f, comHeightOffset, 0f);
        // 给刚体一些阻力，帮助减小空中/无驱动时滑行
        rb.linearDamping = rbLinearDrag;
        rb.angularDamping = rbAngularDrag;

        // 检查数组长度
        if (wheelColliders == null || wheelColliders.Length != 4)
            Debug.LogError("请在 Inspector 中为 RealCarAgent 指定 4 个 WheelCollider（顺序：FL, RL, RR, FR）");
        if (wheels == null || wheels.Length != 4)
            Debug.LogWarning("wheels 长度应为 4，用于可视化轮子。");
    }

    public override void OnEpisodeBegin()
    {
        // 重置刚体速度
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // 重置位姿
        transform.position = startPos;
        transform.rotation = startRot;

        lastForward = transform.forward;
        episodeTimer = 0f;

        // 复位所有轮子控制量
        if (wheelColliders != null)
        {
            for (int i = 0; i < wheelColliders.Length; i++)
            {
                if (wheelColliders[i] != null)
                {
                    wheelColliders[i].motorTorque = 0f;
                    wheelColliders[i].brakeTorque = 0f;
                    wheelColliders[i].steerAngle = 0f;
                }
                if (wheels != null && i < wheels.Length && wheels[i] != null)
                {
                    wheels[i].localRotation = Quaternion.identity;
                }
            }
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 磁场观测（6 个传感器）
        for (int i = 0; i < sensors.Length; i++)
        {
            Vector3 mag = tape.GetMagneticField(sensors[i].position);
            sensor.AddObservation(mag.normalized);
            sensor.AddObservation(Mathf.Clamp01(mag.magnitude / maxField));
        }

        // 车体局部速度 (x 横向, z 前向) — 使用车体局部坐标 z 为前向
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        // 注意：脚本使用的 control frame 是以车体前向为 x 分量的内部计算，观察时返回惯用的横向/前向比
        sensor.AddObservation(localVel.x / Mathf.Max(0.0001f, maxSpeed)); // Unity local x 为右向
        sensor.AddObservation(localVel.z / Mathf.Max(0.0001f, maxSpeed)); // Unity local z 为前向

        // 车头朝向变化（与上一帧比）
        float headingChange = Vector3.SignedAngle(lastForward, transform.forward, Vector3.up) / 180f;
        sensor.AddObservation(Mathf.Clamp(headingChange, -1f, 1f));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 输入：车身坐标系下的前进速度(vx, m/s)、侧向速度(vy, m/s)、角速度(omega, deg/s)
        // 约定：在内部运动学计算时采用车身坐标系向量 (vx_longitudinal, vy_lateral)：
        //  为方便与之前代码一致，内部把“前进”映射为 x 轴正方向（车体前方）
        float vx_cmd = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f) * maxSpeed; // 前向 m/s
        float vy_cmd = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f) * maxSpeed; // 侧向 m/s (右为正)
        float omega_cmd_deg = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f) * maxSteerSpeed; // deg/s
        float omega_cmd = omega_cmd_deg * Mathf.Deg2Rad; // rad/s

        // 轮子在车体坐标系中的位置 (x 前为正, y 侧向右为正)
        Vector2[] wheelPos = new Vector2[4] {
            new Vector2(L/2f,  W/2f),  // FL (前左)  注意 y 为右为正, 左 = +W/2 -> 但几何上前左 y 应是 +W/2 if right positive -> front-left has y=+W/2
            new Vector2(-L/2f, W/2f),  // RL (后左)
            new Vector2(-L/2f, -W/2f), // RR (后右)
            new Vector2(L/2f, -W/2f)   // FR (前右)
        };

        float[] steerAngle = new float[4];       // rad
        float[] wheelLinearSpeed = new float[4]; // m/s (沿车轮滚动方向，正为向前)

        // 计算 Ackermann 转向角（基于 vx 和 omega）
        bool useAckermann = Mathf.Abs(omega_cmd) > 1e-5f;
        float R = 0f; // ICR 半径 (以车体中心为基准, 右侧为正)
        if (useAckermann)
        {
            // 车辆沿车体 x 方向前进，若 vx_cmd 与 omega_cmd 都有符号，R = vx/omega (右为正)
            R = vx_cmd / omega_cmd;
            // 当 vx≈0 且有角速度时，R会很小 -> 处理为原地自转：把前后轮转角设置为 +/- 90deg
            if (Mathf.Abs(vx_cmd) < 1e-4f)
            {
                // 原地自转：设置前后轮相对方向为 ±90deg，速度由 vy/omega 或设置为 0 由驱动决定
                float sign = Mathf.Sign(omega_cmd);
                steerAngle[0] = steerAngle[3] = Mathf.Sign(omega_cmd) * Mathf.PI / 2f; // 前左、前右
                steerAngle[1] = steerAngle[2] = -Mathf.Sign(omega_cmd) * Mathf.PI / 2f; // 后左、后右 (相反)
            }
            else
            {
                // 计算前后左右轮的 ackermann 角
                // 前左 FL: tan(delta_fl) = L / (R - y_fl)
                // 前右 FR: tan(delta_fr) = L / (R - y_fr)  (y_fr is negative)
                float y_fl = wheelPos[0].y;
                float y_fr = wheelPos[3].y;
                float y_rl = wheelPos[1].y;
                float y_rr = wheelPos[2].y;
                // front
                steerAngle[0] = Mathf.Atan(L / (R - y_fl)); // FL
                steerAngle[3] = Mathf.Atan(L / (R - y_fr)); // FR
                // rear steer opposite to front for typical 4-wheel steering (reduce radius)
                steerAngle[1] = -Mathf.Atan(L / (R - y_rl)); // RL
                steerAngle[2] = -Mathf.Atan(L / (R - y_rr)); // RR
            }
        }
        else
        {
            // 直行或微转：所有转角按速度矢量方向计算（考虑 vy）
            for (int i = 0; i < 4; i++)
            {
                float vx_total = vx_cmd;
                float vy_total = vy_cmd;
                steerAngle[i] = Mathf.Atan2(vy_total, vx_total);
            }
        }

        // 若上一段中有未赋 steerAngle（例如原地自转分支已赋），如果仍有未赋则安全赋0
        for (int i = 0; i < 4; i++) if (float.IsNaN(steerAngle[i])) steerAngle[i] = 0f;

        // 计算每个轮子在其朝向上的期望线速度：v_point = v + omega x r ; wheel_speed = dot(v_point, wheel_forward_unit)
        for (int i = 0; i < 4; i++)
        {
            Vector2 r = wheelPos[i];
            // omega x r = omega * [-y, x]
            float vx_rot = -omega_cmd * r.y;
            float vy_rot = omega_cmd * r.x;
            float vx_total = vx_cmd + vx_rot;
            float vy_total = vy_cmd + vy_rot;
            // wheel forward unit in body frame (wheel heading = steerAngle, x is forward)
            Vector2 wf = new Vector2(Mathf.Cos(steerAngle[i]), Mathf.Sin(steerAngle[i]));
            float v_point_dot = vx_total * wf.x + vy_total * wf.y;
            wheelLinearSpeed[i] = Mathf.Clamp(v_point_dot, -maxWheelSpeed, maxWheelSpeed);
        }

        // 应用到 WheelColliders（steerAngle -> wc.steerAngle; wheelLinearSpeed -> motorTorque via P 控制）
        for (int i = 0; i < 4; i++)
        {
            if (wheelColliders == null || wheelColliders.Length <= i) continue;
            WheelCollider wc = wheelColliders[i];
            if (wc == null) continue;

            // 设置转向角（WheelCollider 接受度数）
            wc.steerAngle = Mathf.Rad2Deg * steerAngle[i];

            float wheelRadius = wc.radius;
            float desired_v = wheelLinearSpeed[i];
            float current_rpm = wc.rpm;
            float current_v = current_rpm / 60f * 2f * Mathf.PI * wheelRadius;

            float torque = torqueGain * (desired_v - current_v);
            torque = Mathf.Clamp(torque, -maxMotorTorque, maxMotorTorque);

            // 平滑制动：当期望接近 0 时根据当前速度施加 proportional 制动力
            if (Mathf.Abs(desired_v) < 0.02f)
            {
                wc.motorTorque = 0f;
                float autoBrake = Mathf.Clamp(brakeGain * Mathf.Abs(current_v), 0f, brakeTorqueHigh);
                wc.brakeTorque = autoBrake;
            }
            else
            {
                wc.brakeTorque = 0f;
                wc.motorTorque = torque;
            }
        }

        // 奖励、计时、可视轮子更新
        AddReward(CalculateReward());
        episodeTimer += Time.fixedDeltaTime;
        if (episodeTimer >= maxEpisodeTime) EndEpisode();

        lastForward = transform.forward;
        UpdateVisualWheels();
    }

    private void ApplyBrakeToAll(float brake)
    {
        if (wheelColliders == null) return;
        for (int i = 0; i < wheelColliders.Length; i++)
        {
            if (wheelColliders[i] != null)
            {
                wheelColliders[i].motorTorque = 0f;
                wheelColliders[i].brakeTorque = brake;
            }
        }
    }

    private void UpdateVisualWheels()
    {
        if (wheelColliders == null || wheels == null) return;
        for (int i = 0; i < wheelColliders.Length && i < wheels.Length; i++)
        {
            if (wheelColliders[i] == null || wheels[i] == null) continue;
            Vector3 pos;
            Quaternion rot;
            wheelColliders[i].GetWorldPose(out pos, out rot);
            wheels[i].position = pos;
            wheels[i].rotation = rot;
        }
    }

    private float CalculateReward()
    {
        float reward = 0f;

        // 用车体前向速度作为前进奖励
        float forwardSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);
        reward += w_forward * Mathf.Max(0f, forwardSpeed) * 0.01f;

        // 磁条追踪（前后三组传感器强度均值差）
        float frontAvg = 0f, rearAvg = 0f;
        for (int i = 0; i < 3; i++) frontAvg += tape.GetMagneticField(sensors[i].position).magnitude;
        for (int i = 3; i < 6; i++) rearAvg += tape.GetMagneticField(sensors[i].position).magnitude;
        frontAvg /= 3f; rearAvg /= 3f;
        reward += w_track * (frontAvg - rearAvg);

        // 朝向奖励（与上一帧变化绝对值）
        float headingChange = Vector3.SignedAngle(lastForward, transform.forward, Vector3.up) / 180f;
        reward += w_heading * Mathf.Abs(headingChange);

        return reward;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        cont[0] = Input.GetAxis("Vertical");   // 前向控制
        cont[1] = Input.GetAxis("Horizontal"); // 侧向控制
        cont[2] = 0f;
        if (Input.GetKey(KeyCode.Q)) cont[2] = -1f;
        if (Input.GetKey(KeyCode.E)) cont[2] = 1f;
    }
}
// ...existing code...