// ...existing code...
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class AckermannVehicleController : MonoBehaviour
{
    [Header("Wheel order: FL, RL, RR, FR")]
    public WheelCollider[] wheelColliders = new WheelCollider[4]; // 顺序必须：FL, RL, RR, FR
    public Transform[] wheelMeshes = new Transform[4];            // 对应可视轮（可选）

    [Header("Vehicle geometry (m)")]
    public float wheelBase = 0.76f;   // 前后轴距（m）
    public float trackWidth = 0.47f;  // 轮距（m）

    [Header("Control (body frame)")]
    // vx: 前向速度 (m/s) 沿 transform.forward 正向； vy: 侧向速度 (m/s) 右为正； omega: 偏航角速度 (rad/s)
    [Tooltip("调用 SetControl(vx, vy, omega) 以设置控制量。vx沿transform.forward正，vy右为正，omega为rad/s")]
    public float widget_vx = 0f;
    public float widget_vy = 0f;
    public float widget_omega = 0f;

    [Header("Drive tuning")]
    public float maxWheelSpeed = 8f;       // 单轮线速度上限 (m/s)
    public float maxMotorTorque = 200f;    // 最大电机扭矩 (N·m)
    public float torqueGain = 400f;        // P 控制增益 (torque = gain * (v_des - v_cur))
    public float brakeTorqueHigh = 1500f;  // 强制停止制动力
    public float brakeGain = 800f;         // 自动制动力与当前轮速成比例

    [Header("Options")]
    public bool rearWheelSteer = false;    // 是否启用后轮转向（四轮转向）
    public bool autoComputeGeometry = true; // 自动根据 WheelCollider 位置计算几何尺寸

    Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    void OnValidate()
    {
        if (autoComputeGeometry && wheelColliders != null && wheelColliders.Length == 4)
        {
            if (wheelColliders[0] != null && wheelColliders[1] != null && wheelColliders[2] != null && wheelColliders[3] != null)
            {
                Vector3 pFL = transform.InverseTransformPoint(wheelColliders[0].transform.position);
                Vector3 pRL = transform.InverseTransformPoint(wheelColliders[1].transform.position);
                Vector3 pRR = transform.InverseTransformPoint(wheelColliders[2].transform.position);
                Vector3 pFR = transform.InverseTransformPoint(wheelColliders[3].transform.position);
                wheelBase = Mathf.Abs(pFL.z - pRL.z);
                trackWidth = Mathf.Abs(pFL.x - pFR.x);
            }
        }
    }

    void FixedUpdate()
    {
        ApplyControls(widget_vx, widget_vy, widget_omega);
        UpdateVisualWheels();
    }

    // 外部调用接口：传入车体坐标系下 vx, vy, omega(rad/s)
    public void SetControl(float vx_body, float vy_body, float omega_body)
    {
        widget_vx = vx_body;
        widget_vy = vy_body;
        widget_omega = omega_body;
    }

    void ApplyControls(float vx, float vy, float omega)
    {
        if (wheelColliders == null || wheelColliders.Length != 4) return;

        // 计算每个轮子在车体局部坐标系的位置 (forward = z, right = x)
        // 顺序：0:FL, 1:RL, 2:RR, 3:FR
        Vector2[] r = new Vector2[4];
        for (int i = 0; i < 4; i++)
        {
            if (wheelColliders[i] == null) { r[i] = Vector2.zero; continue; }
            Vector3 localPos = transform.InverseTransformPoint(wheelColliders[i].transform.position);
            r[i] = new Vector2(localPos.z, localPos.x); // r = (forward, right)
        }

        bool turning = Mathf.Abs(omega) > 1e-6f;
        float R = 0f;
        if (turning && Mathf.Abs(vx) > 1e-6f) R = vx / omega; // 右为正

        float[] steerAngles = new float[4]; // rad

        if (turning && Mathf.Abs(vx) > 1e-6f)
        {
            // 常规 Ackermann：只按前轮（或加后轮）计算转角
            float y_fl = r[0].y;
            float y_fr = r[3].y;
            steerAngles[0] = Mathf.Atan2(wheelBase, (R - y_fl)); // FL
            steerAngles[3] = Mathf.Atan2(wheelBase, (R - y_fr)); // FR

            if (rearWheelSteer)
            {
                float y_rl = r[1].y;
                float y_rr = r[2].y;
                steerAngles[1] = -Mathf.Atan2(wheelBase, (R - y_rl)); // RL
                steerAngles[2] = -Mathf.Atan2(wheelBase, (R - y_rr)); // RR
            }
            else
            {
                steerAngles[1] = 0f;
                steerAngles[2] = 0f;
            }
        }
        else if (turning && Mathf.Abs(vx) <= 1e-6f)
        {
            // 原地自转：把前后轮分别转为 +/-90deg（可视需求调整）
            float s = Mathf.Sign(omega);
            steerAngles[0] = s * Mathf.PI * 0.5f;
            steerAngles[3] = s * Mathf.PI * 0.5f;
            steerAngles[1] = -s * Mathf.PI * 0.5f;
            steerAngles[2] = -s * Mathf.PI * 0.5f;
        }
        else
        {
            // 直行或仅侧向速度：所有轮子朝向速度矢量方向
            Vector2 vel = new Vector2(vx, vy);
            float ang = Mathf.Atan2(vel.y, vel.x);
            for (int i = 0; i < 4; i++) steerAngles[i] = ang;
        }

        // 计算每个轮子在其朝向上的期望线速度（考虑角速度对轮点的贡献）
        float[] v_wheel = new float[4];
        for (int i = 0; i < 4; i++)
        {
            float vx_rot = -omega * r[i].y; // omega x r  -> x comp = -omega * y
            float vy_rot = omega * r[i].x;  //           -> y comp =  omega * x
            float vx_total = vx + vx_rot;
            float vy_total = vy + vy_rot;
            Vector2 wf = new Vector2(Mathf.Cos(steerAngles[i]), Mathf.Sin(steerAngles[i])); // wheel forward unit in body (fwd,right)
            float proj = vx_total * wf.x + vy_total * wf.y;
            v_wheel[i] = Mathf.Clamp(proj, -maxWheelSpeed, maxWheelSpeed);
        }

        // 将期望线速度与转角应用到 WheelCollider
        for (int i = 0; i < 4; i++)
        {
            WheelCollider wc = wheelColliders[i];
            if (wc == null) continue;

            // 设置转向角（WheelCollider.steerAngle 为度）
            wc.steerAngle = Mathf.Rad2Deg * steerAngles[i];

            float wheelRadius = Mathf.Max(1e-4f, wc.radius);
            float current_v = wc.rpm / 60f * 2f * Mathf.PI * wheelRadius;
            float desired_v = v_wheel[i];

            float torque = torqueGain * (desired_v - current_v);
            torque = Mathf.Clamp(torque, -maxMotorTorque, maxMotorTorque);

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
    }

    void UpdateVisualWheels()
    {
        if (wheelColliders == null || wheelMeshes == null) return;
        for (int i = 0; i < wheelColliders.Length && i < wheelMeshes.Length; i++)
        {
            var wc = wheelColliders[i];
            var mesh = wheelMeshes[i];
            if (wc == null || mesh == null) continue;
            Vector3 pos; Quaternion rot;
            wc.GetWorldPose(out pos, out rot);
            mesh.position = pos;

            // 确保轮子“竖着”：强制世界 Z 角为 90°
            Vector3 e = rot.eulerAngles;
            e.z = 90f;
            mesh.rotation = Quaternion.Euler(e);
        }
    }
}
// ...existing code...