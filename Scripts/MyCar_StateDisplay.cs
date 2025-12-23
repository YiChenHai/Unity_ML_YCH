using UnityEngine;

/// <summary>
/// 车辆状态显示脚本：在屏幕上显示车辆和轮子的实时信息
/// </summary>
public class MyCar_StateDisplay : MonoBehaviour
{
    [Header("References")]
    public MyCar_Motion myCarMotion;
    public Rigidbody rb;
    public MagneticTape tape;
    [Tooltip("传感器顺序: [0]=前左, [1]=前中, [2]=前右, [3]=后左, [4]=后中, [5]=后右")]
    public Transform[] sensors = new Transform[6];

    [Header("Display Settings")]
    public bool showDebugInfo = true;
    public Vector2 displayPosition = new Vector2(10, 10);
    public Vector2 displaySize = new Vector2(500, 550);

    private static Texture2D _bgTexture; // 静态背景纹理，避免每帧创建

    void Start()
    {
        // 自动查找组件（如果未手动绑定）
        if (myCarMotion == null)
            myCarMotion = GetComponent<MyCar_Motion>();
        
        if (rb == null)
            rb = GetComponent<Rigidbody>();
    }

    void OnGUI()
    {
        if (!showDebugInfo) return;

        // 调试：检查 myCarMotion 是否为空
        if (myCarMotion == null)
        {
            GUI.Label(new Rect(displayPosition.x, displayPosition.y, 300, 50), 
                "ERROR: myCarMotion is null! Please bind MyCar_Motion in Inspector.", 
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
        GUI.DrawTexture(new Rect(displayPosition.x, displayPosition.y, displaySize.x, displaySize.y), _bgTexture);

        GUILayout.BeginArea(new Rect(displayPosition.x, displayPosition.y, displaySize.x, displaySize.y));
        GUILayout.Box("Vehicle & Wheel Info", GUILayout.Width(displaySize.x - 20));

        // ========== 整车信息 ==========
        GUILayout.Label("═══ Vehicle (Body) ═══", GUILayout.Width(displaySize.x - 20));
        
        Vector3 vel = rb != null ? rb.linearVelocity : Vector3.zero;
        float speed = vel.magnitude;
        float forwardSpeed = Vector3.Dot(vel, transform.forward);
        float lateralSpeed = Vector3.Dot(vel, transform.right);
        
        Vector3 angVel = rb != null ? rb.angularVelocity : Vector3.zero;
        float yawRate = angVel.y * Mathf.Rad2Deg; // deg/s

        GUILayout.Label($"Linear Velocity: {speed:F2} m/s (Forward: {forwardSpeed:F2}, Lateral: {lateralSpeed:F2})", 
            GUILayout.Width(displaySize.x - 20));
        GUILayout.Label($"Yaw Rate: {yawRate:F1} deg/s | Position: ({transform.position.x:F2}, {transform.position.z:F2})", 
            GUILayout.Width(displaySize.x - 20));

        GUILayout.Space(10);

        // ========== 各轮子信息（绿色显示）==========
        GUILayout.Label("═══ Wheels ═══", GUILayout.Width(displaySize.x - 20));

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

            GUILayout.Label($"{wheelNames[i]}: Speed={wheelSpeed:F2}m/s Angle={steerDeg:F1}° RPM={wheelRpm:F0} | Motor={motorTorque:F1}Nm Brake={brakeTorque:F1}Nm", 
                greenLabelStyle, GUILayout.Width(displaySize.x - 20));
        }

        GUILayout.Space(10);

        // ========== 控制输入信息 ==========
        GUILayout.Label("═══ Control Input ═══", GUILayout.Width(displaySize.x - 20));
        GUILayout.Label($"Vz(前进): {myCarMotion.vz_input:F3} m/s | Vx(横向): {myCarMotion.vx_input:F3} m/s | Omega: {myCarMotion.omega_input:F3} rad/s", 
            GUILayout.Width(displaySize.x - 20));

        GUILayout.Space(10);

        // ========== 磁传感器信息 ==========
        GUILayout.Label("═══ Magnetic Sensors ═══", GUILayout.Width(displaySize.x - 20));
        
        if (tape != null && sensors != null && sensors.Length == 6)
        {
            float[] sensorValues = new float[6];
            string[] sensorLabels = { "前左", "前中", "前右", "后左", "后中", "后右" };
            
            // 读取传感器数据
            for (int i = 0; i < 6; i++)
            {
                if (sensors[i] != null)
                {
                    Vector3 mag = tape.GetMagneticField(sensors[i].position);
                    sensorValues[i] = mag.magnitude;
                }
            }
            
            // 显示传感器读数（两行显示）
            GUILayout.Label($"{sensorLabels[0]}={sensorValues[0]:F2} | {sensorLabels[1]}={sensorValues[1]:F2} | {sensorLabels[2]}={sensorValues[2]:F2}", 
                GUILayout.Width(displaySize.x - 20));
            GUILayout.Label($"{sensorLabels[3]}={sensorValues[3]:F2} | {sensorLabels[4]}={sensorValues[4]:F2} | {sensorLabels[5]}={sensorValues[5]:F2}", 
                GUILayout.Width(displaySize.x - 20));
            
            // 计算并显示左右差值
            float frontDiff = sensorValues[0] - sensorValues[2];  // 前左 - 前右
            float rearDiff = sensorValues[3] - sensorValues[5];   // 后左 - 后右
            
            GUIStyle diffStyle = new GUIStyle(GUI.skin.label)
            {
                normal = { textColor = Color.yellow },
                fontSize = 12,
                fontStyle = FontStyle.Bold
            };
            
            GUILayout.Label($"前排差值(左-右): {frontDiff:F3} | 后排差值(左-右): {rearDiff:F3}", 
                diffStyle, GUILayout.Width(displaySize.x - 20));
        }
        else
        {
            GUILayout.Label("未配置磁带或传感器", GUILayout.Width(displaySize.x - 20));
        }

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
