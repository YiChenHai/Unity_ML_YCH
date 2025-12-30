using UnityEngine;

/// <summary>
/// 车辆状态显示脚本：在屏幕上显示车辆和轮子的实时信息
/// </summary>
public class MyCar_StateDisplay : MonoBehaviour
{
    [Header("References")]
    public MyCar_Motion myCarMotion;
    public MyCarAgent myCarAgent;
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
        
        if (myCarAgent == null)
            myCarAgent = GetComponent<MyCarAgent>();
        
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
        
        // 显示Agent的速度限制
        if (myCarAgent != null)
        {
            GUILayout.Label($"Agent Limits: ConstVz={myCarAgent.constantForwardSpeed:F2}m/s | MaxVx={myCarAgent.maxLateralSpeed:F2}m/s | MaxOmega={myCarAgent.maxOmegaDeg:F0}°/s", 
                GUILayout.Width(displaySize.x - 20));
        }

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
        
        GUILayout.Space(10);
        
        // ========== 转弯判定状态 ==========
        GUILayout.Label("═══ Tracking (Alignment/Online) ═══", GUILayout.Width(displaySize.x - 20));

        if (myCarAgent != null && tape != null && sensors != null && sensors.Length == 6)
        {
            ComputeTrackingScores(myCarAgent, tape, sensors, out float alignment, out float centerNorm, out float trackQuality);

            // 估算 straightness/hold（用于显示；训练里hold在Agent内部计算）
            float straightness = SmoothStep01(Smooth01((trackQuality - myCarAgent.straightHoldStartQ) / Mathf.Max(1e-4f, 1f - myCarAgent.straightHoldStartQ)));
            float tHold = Smooth01((trackQuality - myCarAgent.straightHoldStartQ) / Mathf.Max(1e-4f, myCarAgent.straightHoldFullQ - myCarAgent.straightHoldStartQ));
            float hold = (myCarAgent.useStraightHold ? myCarAgent.straightHoldStrength * SmoothStep01(tHold) : 0f);

            bool highQuality = trackQuality >= myCarAgent.straightHoldStartQ;
            GUIStyle qualityStyle = new GUIStyle(GUI.skin.label)
            {
                normal = { textColor = highQuality ? Color.cyan : Color.yellow },
                fontSize = 13,
                fontStyle = FontStyle.Bold
            };

            string qualityText = highQuality ? "高质量跟踪 (HIGH-Q)" : "低质量/入弯/修正 (LOW-Q)";
            GUILayout.Label($"当前状态: {qualityText}", qualityStyle, GUILayout.Width(displaySize.x - 20));
            GUILayout.Label($"alignment={alignment:F3} | centerNorm={centerNorm:F3} | trackQuality={trackQuality:F3}", GUILayout.Width(displaySize.x - 20));
            GUILayout.Label($"straightness≈{straightness:F2} | hold≈{hold:F2}", GUILayout.Width(displaySize.x - 20));
            GUILayout.Label($"Agent Action (filtered): vx={myCarAgent.LastActionVx:F2}, omega={myCarAgent.LastActionOmega:F2}", GUILayout.Width(displaySize.x - 20));
        }
        else
        {
            GUILayout.Label("未配置Agent/磁带/传感器", GUILayout.Width(displaySize.x - 20));
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

    private static void ComputeTrackingScores(MyCarAgent agent, MagneticTape tape, Transform[] sensors,
        out float alignment, out float centerNorm, out float trackQuality)
    {
        float[] s = new float[6];
        for (int i = 0; i < 6; i++)
        {
            if (i < sensors.Length && sensors[i] != null && tape != null)
            {
                s[i] = tape.GetMagneticField(sensors[i].position).magnitude;
            }
        }

        float invMax = 1f / Mathf.Max(1e-4f, agent.maxField);

        float frontAbsDiffNorm = Mathf.Abs(s[0] - s[2]) * invMax;
        float rearAbsDiffNorm = Mathf.Abs(s[3] - s[5]) * invMax;

        float frontAlign = ScoreSmall(frontAbsDiffNorm, agent.lrAbsDiffGood, agent.lrAbsDiffBad);
        float rearAlign = ScoreSmall(rearAbsDiffNorm, agent.lrAbsDiffGood, agent.lrAbsDiffBad);
        float rawAlign = Mathf.Min(frontAlign, rearAlign);

        float frontSumNorm = (Mathf.Abs(s[0]) + Mathf.Abs(s[2])) * (0.5f * invMax);
        float rearSumNorm = (Mathf.Abs(s[3]) + Mathf.Abs(s[5])) * (0.5f * invMax);
        float lrSumNorm = Mathf.Min(frontSumNorm, rearSumNorm);
        float trust = SmoothStep01(Smooth01((lrSumNorm - agent.lrMinSumNormForTrust) / Mathf.Max(1e-4f, 1f - agent.lrMinSumNormForTrust)));

        alignment = rawAlign * trust;
        centerNorm = Mathf.Clamp01(Mathf.Min(s[1], s[4]) * invMax);
        trackQuality = alignment * centerNorm;
    }

    private static float ScoreSmall(float x, float good, float bad)
    {
        if (bad <= good) return x <= good ? 1f : 0f;
        float t = Mathf.InverseLerp(good, bad, x);
        return 1f - Mathf.Clamp01(t);
    }

    private static float Smooth01(float x) => Mathf.Clamp01(x);

    private static float SmoothStep01(float x)
    {
        x = Mathf.Clamp01(x);
        return x * x * (3f - 2f * x);
    }
}
