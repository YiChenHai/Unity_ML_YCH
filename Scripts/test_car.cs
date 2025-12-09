// ...existing code...
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class TestCar : MonoBehaviour
{
    [Header("Wheel order: FL, RL, RR, FR")]
    public WheelCollider[] wheelColliders = new WheelCollider[4]; // 按顺序赋值：FL, RL, RR, FR
    public Transform[] wheelMeshes = new Transform[4];            // 可视化轮子

    [Header("Vehicle geometry (m)")]
    public float wheelBase = 0.76f;   // 轴距 L
    public float trackWidth = 0.47f;  // 轮距 W

    [Header("Control inputs (body frame)")]
    // 在车身坐标系中输入：vx 沿 transform.forward 正向（m/s），vy 右为正（m/s），omega 绕 up (rad/s)
    public float vx_input = 0f;
    public float vy_input = 0f;
    public float omega_input = 0f;

    [Header("Kinematic scaling & deadzone")]
    public float inputScaleVx = 1f;      // 输入缩放（可根据上位机单位调整）
    public float inputScaleVy = 1f;
    public float inputScaleOmega = 1f;
    public float deadzone = 0.1f;        // 当所有输入均在死区内时认为静止

    [Header("Wheel / Drive")]
    public float maxWheelLinearSpeed = 4.0f;  // MAX_MOTOR_SPEED (m/s)
    public float maxMotorTorque = 200f;       // 电机扭矩极限 (N·m)
    public float torqueGain = 400f;           // P 控制增益 (N·m per (m/s))
    public float brakeTorqueHigh = 1000f;
    public float brakeGain = 800f;            // proportional brake

    [Header("Visual options")]
    public bool forceWheelZto90 = true;       // 保证可视轮子的世界Z角为90°

    Rigidbody rb;

    // 输出（供外部读取）— 注意：这些是“实际应用到 WheelCollider 上”的输出（已做前后互换映射）
    [HideInInspector] public float[] steerAngles = new float[4]; // rad, 按 wheelColliders 顺序
    [HideInInspector] public float[] wheelSpeeds = new float[4]; // m/s (线速度，沿轮滚动方向)，按 wheelColliders 顺序

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        // 读取并缩放输入
        float vx = vx_input * inputScaleVx;
        float vy = vy_input * inputScaleVy;
        float omega = omega_input * inputScaleOmega;

        // 调用解析与驱动（内部计算依然按 FL,RL,RR,FR 顺序）
        ComputeKinematicsAndApply(vx, vy, omega);

        // 更新可视轮
        UpdateVisualWheels();
    }

    // 外部接口：设置控制量（body frame）
    public void SetControl(float vx, float vy, float omega)
    {
        vx_input = vx;
        vy_input = vy;
        omega_input = omega;
    }

    void ComputeKinematicsAndApply(float vx, float vy, float omega)
    {
        // 死区判断（若都很小则清零输出并施加刹车）
        if (Mathf.Abs(vx) < deadzone && Mathf.Abs(vy) < deadzone && Mathf.Abs(omega) < deadzone)
        {
            // 清零并刹车（按 wheelColliders 实际顺序）
            for (int j = 0; j < 4; j++)
            {
                steerAngles[j] = 0f;
                wheelSpeeds[j] = 0f;
                if (wheelColliders != null && j < wheelColliders.Length && wheelColliders[j] != null)
                {
                    wheelColliders[j].motorTorque = 0f;
                    wheelColliders[j].brakeTorque = brakeTorqueHigh;
                    wheelColliders[j].steerAngle = 0f;
                }
            }
            return;
        }

        // 轮子相对于车体中心的位置（按照你的约定：FL, RL, RR, FR）
        Vector2[] wheelPos = new Vector2[4] {
            new Vector2( wheelBase/2f,  trackWidth/2f),  // FL (前左)
            new Vector2(-wheelBase/2f,  trackWidth/2f),  // RL (后左)
            new Vector2(-wheelBase/2f, -trackWidth/2f),  // RR (后右)
            new Vector2( wheelBase/2f, -trackWidth/2f)   // FR (前右)
        };

        // 临时存放按解析顺序计算的角度和速度（解析索引：0=FL,1=RL,2=RR,3=FR）
        float[] kinSteer = new float[4];
        float[] kinSpeed = new float[4];

        // 按 txt 中的解析：先计算每轮点的速度矢量，然后 steer=atan2(vy_total, vx_total)，speed = magnitude
        for (int i = 0; i < 4; i++)
        {
            float vx_rot = -omega * wheelPos[i].y;  // -ω * y
            float vy_rot =  omega * wheelPos[i].x;  //  ω * x

            float vx_total = vx + vx_rot;
            float vy_total = vy + vy_rot;

            float ang = Mathf.Atan2(vy_total, vx_total); // rad
            float vmag = Mathf.Sqrt(vx_total * vx_total + vy_total * vy_total);

            kinSteer[i] = ang;
            kinSpeed[i] = vmag;
        }

        // 调整转向角范围到 [-pi/2, pi/2]，如果超过则翻转速度符号（与txt一致）
        for (int i = 0; i < 4; i++)
        {
            if (kinSteer[i] > Mathf.PI / 2f)
            {
                kinSteer[i] -= Mathf.PI;
                kinSpeed[i] = -kinSpeed[i];
            }
            else if (kinSteer[i] < -Mathf.PI / 2f)
            {
                kinSteer[i] += Mathf.PI;
                kinSpeed[i] = -kinSpeed[i];
            }
        }

        // 速度归一化，如果任意轮速度超过 maxWheelLinearSpeed 则按比例缩放
        float max_speed_abs = 0f;
        for (int i = 0; i < 4; i++) max_speed_abs = Mathf.Max(max_speed_abs, Mathf.Abs(kinSpeed[i]));
        if (max_speed_abs > maxWheelLinearSpeed && max_speed_abs > 0f)
        {
            float scale = maxWheelLinearSpeed / max_speed_abs;
            for (int i = 0; i < 4; i++) kinSpeed[i] *= scale;
        }

        // 映射：计算索引（0=FL,1=RL,2=RR,3=FR） -> 实际 WheelCollider 索引
        // 要求：把计算出的前左与后左互换，右侧同理
        int[] map = new int[4] { 1, 0, 3, 2 };

        // 构造应用数组（按 wheelColliders 实际顺序）
        float[] appliedSteer = new float[4];
        float[] appliedSpeed = new float[4];
        for (int i = 0; i < 4; i++)
        {
            int tgt = map[i]; // 将第 i 个计算结果写入到 wheelColliders[tgt]
            appliedSteer[tgt] = kinSteer[i];
            appliedSpeed[tgt] = kinSpeed[i];
        }

        // 保存到对外公开的 steerAngles/wheelSpeeds（按 wheelColliders 顺序）
        for (int j = 0; j < 4; j++)
        {
            steerAngles[j] = appliedSteer[j];
            wheelSpeeds[j] = appliedSpeed[j];
        }

        // 应用到 WheelCollider：设置 steerAngle (deg) 和 motorTorque / brakeTorque（按 wheelColliders 顺序）
        for (int j = 0; j < 4; j++)
        {
            if (wheelColliders == null || j >= wheelColliders.Length) continue;
            WheelCollider wc = wheelColliders[j];
            if (wc == null) continue;

            // 设置转向角（WheelCollider 接受度数）
            wc.steerAngle = Mathf.Rad2Deg * appliedSteer[j];

            // 当前轮线速度 (m/s) 从 rpm 计算
            float wheelRadius = Mathf.Max(1e-4f, wc.radius);
            float current_v = wc.rpm / 60f * 2f * Mathf.PI * wheelRadius;

            float desired_v = appliedSpeed[j];

            // P 控制扭矩
            float torque = torqueGain * (desired_v - current_v);
            torque = Mathf.Clamp(torque, -maxMotorTorque, maxMotorTorque);

            // 如果期望速度非常小，则施加比例制动力以更快停车
            if (Mathf.Abs(desired_v) < 0.01f)
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
    }

    void UpdateVisualWheels()
    {
        if (wheelColliders == null || wheelMeshes == null) return;
        for (int i = 0; i < wheelColliders.Length && i < wheelMeshes.Length; i++)
        {
            var wc = wheelColliders[i];
            var mesh = wheelMeshes[i];
            if (wc == null || mesh == null) continue;
            Vector3 pos;
            Quaternion rot;
            wc.GetWorldPose(out pos, out rot);

            mesh.position = pos;

            if (forceWheelZto90)
            {
                Vector3 e = rot.eulerAngles;
                e.z = 90f;
                mesh.rotation = Quaternion.Euler(e);
            }
            else
            {
                mesh.rotation = rot;
            }
        }
    }
}
// ...existing code...