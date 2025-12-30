using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// 训练用智能体：四轮独立转向/驱动底盘的“磁条跟踪”任务。
/// 设计目标：
/// 1) 对齐磁条：前/后 左右传感器差值足够小；中心传感器用于离线判断。
/// 2) 运动平滑：直线仅Vz(固定前进速度)，车体侧向/摆头小；转弯动作连贯不抖动。
///
/// 说明：
/// - 本脚本只依赖 MagneticTape.GetMagneticField(Vector3).magnitude 以及你现有 MyCar_Motion.SetControl(vz,vx,omega)
/// - 不依赖轮子角度/轮速等内部实现（避免与你的运动学脚本耦合）。
/// </summary>
public class MyCarAgentTrain : Agent
{
    [Header("Refs")]
    public MagneticTape tape;

    [Tooltip("传感器顺序: [0]=前左, [1]=前中, [2]=前右, [3]=后左, [4]=后中, [5]=后右")]
    public Transform[] sensors = new Transform[6];

    public Rigidbody rb;
    public MyCar_Motion myCarMotion;

    [Header("Control (body frame: X=右, Z=前)")]
    [Tooltip("固定前进速度 Vz (m/s)")]
    public float vzConst = 0.2f;
    [Tooltip("最大侧向速度 |Vx| 上限 (m/s)")]
    public float vxMax = 0.8f;
    [Tooltip("最大自转角速度 |omega| 上限 (deg/s)")]
    public float omegaMaxDeg = 100f;

    [Header("Magnetic field normalization")]
    [Tooltip("用于把磁场强度归一化到[0,1]的参考值")]
    public float fieldMax = 8f;

    [Header("Episode")]
    public float maxEpisodeTime = 20f;

    [Header("Timeout")]
    [Tooltip("超时终止时给予的惩罚（一般取负值）。若不想惩罚可设为0")]
    public float timeoutPenalty = -1f;

    [Header("Termination")]
    [Tooltip("中心传感器归一化强度 < 阈值 时判定离线并终止")]
    public float centerOfflineThresholdNorm = 0.20f;

    [Header("Action shaping")]
    [Tooltip("动作死区：小于该值直接置0，降低微抖")]
    [Range(0f, 0.2f)]
    public float actionDeadzone = 0.03f;

    [Tooltip("动作一阶低通滤波时间常数(秒)。越大越平滑")]
    public float actionFilterTau = 0.08f;

    [Tooltip("动作速率限制：vx动作(归一化)每秒最大变化量")]
    public float maxActionRateVx = 8f;

    [Tooltip("动作速率限制：omega动作(归一化)每秒最大变化量")]
    public float maxActionRateOmega = 10f;

    [Header("Reward weights")]
    [Tooltip("在线奖励：中心信号越强越好")]
    public float wOnTrack = 1.0f;

    [Tooltip("对齐奖励：左右差归一化越小越好")]
    public float wAlign = 1.2f;

    [Tooltip("姿态对齐奖励：前后横向误差一致(减少蛇形)")]
    public float wHeading = 0.6f;

    [Tooltip("固定前进速度奖励：鼓励速度沿车体前向且接近 vzConst")]
    public float wForwardSpeed = 0.2f;

    [Tooltip("侧向速度惩罚：抑制直线漂移")]
    public float wLatVel = 0.4f;

    [Tooltip("摆头惩罚：抑制高角速度(直线摆轮通常伴随高yaw)")]
    public float wYawRate = 0.4f;

    [Tooltip("动作幅度惩罚：避免无意义大动作")]
    public float wActionMag = 0.12f;

    [Tooltip("动作变化率惩罚：抑制抖动/来回修正")]
    public float wActionRate = 0.10f;

    [Header("Start pose")]
    public Vector3 startPos = new Vector3(1f, 0.25f, -1.233f);
    public Vector3 startEuler = Vector3.zero;

    [Header("Debug")]
    public bool enableDebugLog;
    public int debugLogEveryNFrames = 30;

    // --- runtime ---
    private readonly float[] sensorRaw = new float[6];
    private readonly float[] sensorNorm = new float[6];

    private float episodeTimer;

    private float filteredAx;   // normalized action for Vx
    private float filteredAom;  // normalized action for omega
    private float lastFilteredAx;
    private float lastFilteredAom;

    public override void Initialize()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();

        // Clamp to safe values
        vzConst = Mathf.Max(0f, vzConst);
        vxMax = Mathf.Max(1e-4f, vxMax);
        omegaMaxDeg = Mathf.Max(1e-4f, omegaMaxDeg);
        fieldMax = Mathf.Max(1e-4f, fieldMax);
        maxEpisodeTime = Mathf.Max(0.1f, maxEpisodeTime);
        // timeoutPenalty 不强制为负；允许设为0表示“只终止不惩罚”。
        actionFilterTau = Mathf.Max(1e-4f, actionFilterTau);
        debugLogEveryNFrames = Mathf.Max(1, debugLogEveryNFrames);
    }

    public override void OnEpisodeBegin()
    {
        episodeTimer = 0f;

        transform.position = startPos;
        transform.rotation = Quaternion.Euler(startEuler);

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        filteredAx = 0f;
        filteredAom = 0f;
        lastFilteredAx = 0f;
        lastFilteredAom = 0f;

        // 初始给一个“只前进不转向”的控制
        if (myCarMotion != null) myCarMotion.SetControl(vzConst, 0f, 0f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        UpdateSensors();

        // 1) 六个磁传感器强度(归一化)
        // 说明：本任务假设传感器只能上传“标量强度”，不依赖磁场方向信息。
        for (int i = 0; i < 6; i++) sensor.AddObservation(sensorNorm[i]);

        // 2) 车体运动状态（归一化）
        // 说明：加入速度/角速度能显著减少“只靠当前磁场做抖动修正”的策略，利于学到连续控制。
        Vector3 velWorld = rb != null ? rb.linearVelocity : Vector3.zero;
        Vector3 velLocal = transform.InverseTransformDirection(velWorld);

        sensor.AddObservation(ClampSigned(velLocal.z / Mathf.Max(1e-4f, vzConst), 2f));
        sensor.AddObservation(ClampSigned(velLocal.x / Mathf.Max(1e-4f, vxMax), 2f));

        float omegaMaxRad = omegaMaxDeg * Mathf.Deg2Rad;
        float yawRate = rb != null ? rb.angularVelocity.y : 0f;
        sensor.AddObservation(ClampSigned(yawRate / Mathf.Max(1e-4f, omegaMaxRad), 2f));

        // 3) 派生跟踪特征：横向误差(前/后) + 在线强度
        // eFront/eRear 是“左右差占比”，天然归一化，并抑制弱信号下差值放大。
        ComputeTrackFeatures(
            out float eFront,
            out float eRear,
            out float centerMinNorm,
            out float alignQuality,
            out float headingQuality);

        // 横向误差 eFront/eRear 本身在[-1,1]，直接喂给网络
        sensor.AddObservation(eFront);
        sensor.AddObservation(eRear);
        sensor.AddObservation(centerMinNorm);
        sensor.AddObservation(alignQuality);
        sensor.AddObservation(headingQuality);

        // 4) 上一时刻滤波动作（让策略更容易学到“连续控制”）
        // 给策略提供“刚才输出了什么”的上下文，可降低抖动并加快收敛。
        sensor.AddObservation(filteredAx);
        sensor.AddObservation(filteredAom);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float dt = Time.fixedDeltaTime;
        episodeTimer += dt;

        UpdateSensors();
        ComputeTrackFeatures(
            out float eFront,
            out float eRear,
            out float centerMinNorm,
            out float alignQuality,
            out float headingQuality);

        // --- Termination: 离线 ---
        // 使用前/后中心传感器的最小值：只要任一中心掉线，就认为车身已偏离或姿态不可用。
        if (centerMinNorm < centerOfflineThresholdNorm)
        {
            AddReward(-3f);
            EndEpisode();
            return;
        }

        if (episodeTimer >= maxEpisodeTime)
        {
            // --- Termination: 超时 ---
            // 超时通常表示策略进入了“低效/震荡但不脱轨”的状态；给轻微惩罚可推动更快完成稳定跟踪。
            if (timeoutPenalty != 0f) AddReward(timeoutPenalty);
            if (enableDebugLog)
            {
                Debug.Log($"Episode Ended: timeout. t={episodeTimer:F2}s, totalReward={GetCumulativeReward():F3}");
            }
            EndEpisode();
        }

        // --- Actions (normalized): a0=Vx, a1=omega ---
        float ax = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float aom = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        if (Mathf.Abs(ax) < actionDeadzone) ax = 0f;
        if (Mathf.Abs(aom) < actionDeadzone) aom = 0f;

        // 低通滤波 + 速率限制（保证“直线不抖、转弯一气呵成”）
        // 低通：把策略输出当作“期望”，通过一阶惯性平滑。
        // 限速：限制每秒最大变化量，避免策略用高频抖动“取巧”。
        float alpha = 1f - Mathf.Exp(-dt / Mathf.Max(1e-4f, actionFilterTau));

        float axTarget = Mathf.Lerp(filteredAx, ax, alpha);
        float aomTarget = Mathf.Lerp(filteredAom, aom, alpha);

        float axMaxStep = Mathf.Max(0f, maxActionRateVx) * dt;
        float aomMaxStep = Mathf.Max(0f, maxActionRateOmega) * dt;

        filteredAx = Mathf.MoveTowards(filteredAx, axTarget, axMaxStep);
        filteredAom = Mathf.MoveTowards(filteredAom, aomTarget, aomMaxStep);

        // 下发控制：Vz 固定；输出 Vx 与 omega
        // 说明：Vz 固定等价于把任务聚焦在“横向纠偏 + 航向控制”，满足你“直线仅Vz=0.2m/s”的约束。
        float vx = filteredAx * vxMax;
        float omega = filteredAom * omegaMaxDeg * Mathf.Deg2Rad;
        if (myCarMotion != null) myCarMotion.SetControl(vzConst, vx, omega);

        // --- Reward shaping ---
        // 在线：中心信号强（越强越靠近磁条中心区域，也更可信）
        float rOnTrack = centerMinNorm;

        // 对齐：左右差小（用 eFront/eRear 的绝对值衡量）
        // e 越接近 0 表示左右更对称，车体更对齐磁条。
        float alignErr = 0.5f * (Mathf.Abs(eFront) + Mathf.Abs(eRear)); // [0,1]
        float rAlign = 1f - alignErr;

        // 航向：前后误差一致 => 减少蛇形，转弯更“顺”
        // 直观解释：若车身存在偏航角，前后截面看到的横向误差会不一致；
        // 因此压小 |eFront-eRear| 相当于压小航向误差，抑制“左右摆头→过冲→反向纠偏”的蛇形。
        float headingErr = Mathf.Abs(eFront - eRear); // [0,2]，但实际常在[0,1]
        float rHeading = 1f - Mathf.Clamp01(0.5f * headingErr);

        // 速度：鼓励沿前向且接近 vzConst
        Vector3 velWorld = rb != null ? rb.linearVelocity : Vector3.zero;
        float vForward = Vector3.Dot(velWorld, transform.forward);
        float rForward = Mathf.Clamp01(vForward / Mathf.Max(1e-4f, vzConst));

        // 平滑：抑制侧向速度 & yaw 角速度
        // 这两项是“直线不摆轮”的车体层面代理指标：侧滑/摆头越大，越可能对应频繁修正或过冲。
        Vector3 velLocal = transform.InverseTransformDirection(velWorld);
        float latVelNorm = ClampSigned(velLocal.x / Mathf.Max(1e-4f, vxMax), 2f);

        float omegaMaxRad = omegaMaxDeg * Mathf.Deg2Rad;
        float yawRate = rb != null ? rb.angularVelocity.y : 0f;
        float yawRateNorm = ClampSigned(yawRate / Mathf.Max(1e-4f, omegaMaxRad), 2f);

        // 动作幅度/变化率惩罚
        // 幅度惩罚：避免无意义大动作；变化率惩罚：抑制高频来回修正（抖动）。
        float actionMag = filteredAx * filteredAx + filteredAom * filteredAom;
        float dAction = Mathf.Abs(filteredAx - lastFilteredAx) + Mathf.Abs(filteredAom - lastFilteredAom);

        float rewardPerSec = 0f;
        rewardPerSec += wOnTrack * rOnTrack;
        rewardPerSec += wAlign * rAlign;
        rewardPerSec += wHeading * rHeading;
        rewardPerSec += wForwardSpeed * rForward;

        rewardPerSec -= wLatVel * (latVelNorm * latVelNorm);
        rewardPerSec -= wYawRate * (yawRateNorm * yawRateNorm);
        rewardPerSec -= wActionMag * actionMag;
        rewardPerSec -= wActionRate * (dAction * dAction);

        AddReward(rewardPerSec * dt);

        lastFilteredAx = filteredAx;
        lastFilteredAom = filteredAom;

        if (enableDebugLog && (debugLogEveryNFrames <= 1 || Time.frameCount % debugLogEveryNFrames == 0))
        {
            Debug.Log($"center={centerMinNorm:F2}, alignQ={alignQuality:F2}, headQ={headingQuality:F2}, eF={eFront:F2}, eR={eRear:F2}, act=({filteredAx:F2},{filteredAom:F2}), latV={velLocal.x:F3}, yaw={yawRate:F3}");
        }

    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // 可选：留空或按需写键盘控制
    }

    private void UpdateSensors()
    {
        float invMax = 1f / Mathf.Max(1e-4f, fieldMax);

        for (int i = 0; i < 6; i++)
        {
            float mag = 0f;
            if (tape != null && sensors != null && i < sensors.Length && sensors[i] != null)
            {
                // 关键前提：只使用磁场“强度标量”，不使用方向/向量分量。
                mag = tape.GetMagneticField(sensors[i].position).magnitude;
            }

            sensorRaw[i] = mag;
            sensorNorm[i] = Mathf.Clamp01(mag * invMax);
        }
    }

    /// <summary>
    /// 基于 6 个传感器信号构造“横向误差 + 在线强度 + 质量指标”。
    /// eFront/eRear：用左右差相对于总强度的比例，稳定且天然归一化。
    /// </summary>
    private void ComputeTrackFeatures(
        out float eFront,
        out float eRear,
        out float centerMinNorm,
        out float alignQuality,
        out float headingQuality)
    {
        float fl = sensorNorm[0];
        float fc = sensorNorm[1];
        float fr = sensorNorm[2];
        float rl = sensorNorm[3];
        float rc = sensorNorm[4];
        float rr = sensorNorm[5];

        // e in [-1,1] (右强为正)
        // 采用差值/总和而不是纯差值：
        // - 纯差值在弱信号区域会被“放大”，导致策略在边缘抖动；
        // - 差值占比对信号幅值变化更鲁棒，更适合训练。
        eFront = SignedLR(fl, fr, fc);
        eRear = SignedLR(rl, rr, rc);

        // 在线强度：前/后中心都要强才算可靠。
        centerMinNorm = Mathf.Min(fc, rc);

        float alignErr = 0.5f * (Mathf.Abs(eFront) + Mathf.Abs(eRear));
        alignQuality = 1f - alignErr;

        float headingErr = Mathf.Abs(eFront - eRear);
        headingQuality = 1f - Mathf.Clamp01(0.5f * headingErr);
    }

    private static float SignedLR(float left, float right, float center)
    {
        // 用总强度做归一化，避免“弱信号下差值放大”
        float denom = Mathf.Max(1e-4f, left + right + 0.5f * center);
        float e = (right - left) / denom;
        return Mathf.Clamp(e, -1f, 1f);
    }

    private static float ClampSigned(float x, float limitAbs)
    {
        return Mathf.Clamp(x, -Mathf.Abs(limitAbs), Mathf.Abs(limitAbs));
    }
}
