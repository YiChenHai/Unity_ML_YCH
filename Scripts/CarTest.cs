using UnityEngine;

public class CarController : MonoBehaviour
{
    public float vx;  // Z轴方向速度（车头前进方向）
    public float vy;  // X轴方向速度（车身的横向速度）
    public float omega;  // 角速度（车体自转速度）

    public float WHEEL_BASE = 0.76f;  // 轴距
    public float TRACK_WIDTH = 0.47f;  // 轮距
    public float MAX_MOTOR_SPEED = 4.0f;  // 最大电机速度

    public WheelCollider wheelFL;  // 前左车轮的 WheelCollider
    public WheelCollider wheelFR;  // 前右车轮的 WheelCollider
    public WheelCollider wheelRL;  // 后左车轮的 WheelCollider
    public WheelCollider wheelRR;  // 后右车轮的 WheelCollider

    public Transform wheelFLMesh;  // 前左车轮网格
    public Transform wheelFRMesh;  // 前右车轮网格
    public Transform wheelRLMesh;  // 后左车轮网格
    public Transform wheelRRMesh;  // 后右车轮网格

    private float[] steer_motor = new float[4];  // 四个车轮的转向角度（弧度）
    private float[] speed_motor = new float[4];  // 四个车轮的速度（m/s）

    private Rigidbody rb;




    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        Debug.Log("CarController 已初始化");

        // 设置合适的质心
        rb.centerOfMass = new Vector3(0f, -0.5f, 0f); // 将重心稍微设置低一些
    }

    private void FixedUpdate()
    {
        // 死区处理
        float DEADZONE = 0.1f;
        if (Mathf.Abs(vx) < DEADZONE && Mathf.Abs(vy) < DEADZONE && Mathf.Abs(omega) < DEADZONE)
        {
            for (int i = 0; i < 4; i++)
            {
                speed_motor[i] = 0;
                steer_motor[i] = 0;
            }
            return;
        }

        // 打印车体的 Rigidbody 速度
        Debug.Log($"车体 Rigidbody 速度: {rb.linearVelocity}");

        // 轮子位置顺序调整为：前左、后左、后右、前右
        Vector2[] wheel_pos = new Vector2[4]
        {
            new Vector2(WHEEL_BASE / 2, TRACK_WIDTH / 2),  // 前左
            new Vector2(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // 后左
            new Vector2(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), // 后右
            new Vector2(WHEEL_BASE / 2, -TRACK_WIDTH / 2)  // 前右
        };

        // 计算每个轮子的速度向量（转速对应车轮x方向角速度，转弯角度对应y方向角度）
        for (int i = 0; i < 4; i++)
        {
            // 根据车体坐标系来计算车轮的速度分量
            // vx 代表车身的 Z 方向速度
            // vy 代表车身的 X 方向速度
            // omega 是车身的自转角速度（绕 Y 轴转动）

            // 计算车轮x方向的角速度（转速）
            float vx_rot = -omega * wheel_pos[i].y;  // x方向的旋转速度分量（-ω * y）
            float vy_rot = omega * wheel_pos[i].x;   // y方向的旋转速度分量（ω * x）
            float vx_total = vx + vx_rot;            // x方向总速度（平移 + 旋转）
            float vy_total = vy + vy_rot;            // y方向总速度（平移 + 旋转）

            // 计算车轮的转向角度（y方向角度）
            steer_motor[i] = Mathf.Atan2(vy_total, vx_total);  // 转向角度（弧度）

            // 计算车轮的速度（转速）
            speed_motor[i] = Mathf.Sqrt(vx_total * vx_total + vy_total * vy_total);  // 计算速度大小

            // 打印每个车轮的速度和转向角度
            Debug.Log($"车轮 {i}: vx_total = {vx_total}, vy_total = {vy_total}, steer_motor = {steer_motor[i]}, speed_motor = {speed_motor[i]}");
        }

        // 调整转向角度范围 [-π/2, π/2]
        for (int i = 0; i < 4; i++)
        {
            if (steer_motor[i] > Mathf.PI / 2)
            {
                steer_motor[i] -= Mathf.PI;  // 减去180°
                speed_motor[i] = -speed_motor[i];  // 速度取反
            }
            else if (steer_motor[i] < -Mathf.PI / 2)
            {
                steer_motor[i] += Mathf.PI;  // 加上180°
                speed_motor[i] = -speed_motor[i];  // 速度取反
            }
        }

        // 速度归一化处理
        float max_speed = Mathf.Max(Mathf.Abs(speed_motor[0]), Mathf.Abs(speed_motor[1]), Mathf.Abs(speed_motor[2]), Mathf.Abs(speed_motor[3]));
        if (max_speed > MAX_MOTOR_SPEED)
        {
            float scale = MAX_MOTOR_SPEED / max_speed;  // 缩放比例
            for (int i = 0; i < 4; i++)
            {
                speed_motor[i] *= scale;  // 所有轮子的速度按比例缩小
            }
        }

        // 打印归一化后的速度
        Debug.Log($"最大速度: {max_speed}, 缩放后的车轮速度: {string.Join(", ", speed_motor)}");

        // 将计算得到的转向角和速度应用到四个车轮的 WheelCollider 中
        // 控制四个车轮的转向角度
        wheelFL.steerAngle = steer_motor[0] * Mathf.Rad2Deg;  // 将弧度转换为度
        wheelFR.steerAngle = steer_motor[1] * Mathf.Rad2Deg;
        wheelRL.steerAngle = steer_motor[2] * Mathf.Rad2Deg;
        wheelRR.steerAngle = steer_motor[3] * Mathf.Rad2Deg;

        // 控制四个车轮的速度

        wheelFL.motorTorque = speed_motor[0]; 
        wheelFR.motorTorque = speed_motor[1]; 
        wheelRL.motorTorque = speed_motor[2]; 
        wheelRR.motorTorque = speed_motor[3]; 
  
//
        // 打印车体的速度
        Debug.Log($"车体速度: {rb.linearVelocity}");

        // 同步四个轮子的网格位置和旋转
        UpdateWheelMeshes();
    }

    // 更新每个车轮的视觉网格位置和旋转
    void UpdateWheelMeshes()
    {
        UpdateWheelMesh(wheelFL, wheelFLMesh);
        UpdateWheelMesh(wheelFR, wheelFRMesh);
        UpdateWheelMesh(wheelRL, wheelRLMesh);
        UpdateWheelMesh(wheelRR, wheelRRMesh);
    }

    void UpdateWheelMesh(WheelCollider wheelCollider, Transform wheelMesh)
    {
        if (wheelMesh == null || wheelCollider == null) return;

        // 获取物理车轮的位置和旋转
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);

        // 更新网格的位置和旋转
        wheelMesh.position = pos;
        wheelMesh.rotation = rot;

        // 强制车轮的Z轴角度为90°，保持车轮垂直
        wheelMesh.rotation = Quaternion.Euler(wheelMesh.rotation.eulerAngles.x, wheelMesh.rotation.eulerAngles.y, 90f);

        // 打印车轮网格的位置和旋转
        Debug.Log($"车轮网格位置: {wheelMesh.position}, 旋转角度: {wheelMesh.rotation.eulerAngles}");
    }
}
