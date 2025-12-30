using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class MyCarAgent : Agent
{
    [Header("Refs")]
    public MagneticTape tape;

    [Tooltip("传感器顺序: [0]=前左, [1]=前中, [2]=前右, [3]=后左, [4]=后中, [5]=后右")]
    public Transform[] sensors = new Transform[6];

    public Rigidbody rb;
    public MyCar_Motion myCarMotion;

    [Header("Control (body frame: X=右, Z=前)")]
    [Tooltip("直线要求：仅Vz=0.2 m/s")]
    public float constantForwardSpeed = 0.2f;
    public float maxLateralSpeed = 0.8f;
    public float maxOmegaDeg = 100f;

    [Header("Normalization")]
    public float maxField = 8f;

    [Header("Episode")]
    public float maxEpisodeTime = 20f;

    [Header("Termination")]
    [Tooltip("中心传感器低于该值判定离线/脱轨并终止")]
    public float derailThreshold = 2f;

    [Header("Tracking score (核心：对齐 + 在线)")]
    [Tooltip("左右差值(归一化到maxField) <= good 时认为对齐很好")]
    public float lrAbsDiffGood = 0.06f;
    [Tooltip("左右差值(归一化到maxField) >= bad 时认为对齐很差")]
    public float lrAbsDiffBad = 0.22f;
    [Tooltip("左右两侧信号强度过低时，不信任对齐（用于抑制弱信号放大）")]
    public float lrMinSumNormForTrust = 0.20f;

    [Header("Action shaping (目标：直线轮子不抖、转弯一气呵成)")]
    [Tooltip("动作死区（归一化动作绝对值小于该值直接置0）")]
    public float actionDeadzone = 0.03f;
    [Tooltip("动作输出一阶低通滤波时间常数(秒)。越小越实时，越大越平滑")]
    public float actionFilterTau = 0.05f;
    [Tooltip("动作速率限制：vx动作(归一化)每秒最大变化量")]
    public float maxActionRateVx = 8f;
    [Tooltip("动作速率限制：omega动作(归一化)每秒最大变化量")]
    public float maxActionRateOmega = 10f;

    [Header("Straight hold (连续门控，不区分转弯模式)")]
    public bool useStraightHold = true;
    [Tooltip("当trackQuality达到该值开始强制收敛到vx=0、omega=0")]
    public float straightHoldStartQ = 0.75f;
    [Tooltip("当trackQuality达到该值时直线保持达到最大强度")]
    public float straightHoldFullQ = 0.90f;
    [Range(0f, 1f)]
    public float straightHoldStrength = 0.95f;

    [Header("Reward weights")]
    [Tooltip("跟踪质量奖励：trackQuality(对齐×在线)")]
    public float wTrackQuality = 1.5f;
    [Tooltip("前进速度奖励：鼓励维持Vz")]
    public float wForward = 0.3f;

    [Tooltip("侧向速度惩罚（实际localVel.x），直线时更重")]
    public float wLatVelStraight = 0.70f;
    public float wLatVelTurn = 0.20f;
    [Tooltip("角速度惩罚（实际rb.angularVelocity.y），直线时更重")]
    public float wYawRateStraight = 0.70f;
    public float wYawRateTurn = 0.25f;

    [Tooltip("轮子转向角幅度惩罚（直线摆轮核心约束）")]
    public float wSteerAbsStraight = 0.90f;
    public float wSteerAbsTurn = 0.25f;
    [Tooltip("轮子转向角变化率惩罚（抑制抖动/来回修正）")]
    public float wSteerRateStraight = 0.70f;
    public float wSteerRateTurn = 0.25f;

    [Tooltip("动作幅度惩罚（基于滤波后的动作），直线时更重")]
    public float wActionMagStraight = 0.55f;
    public float wActionMagTurn = 0.15f;
    [Tooltip("动作变化率惩罚（基于滤波后的动作变化），直线时更重")]
    public float wActionRateStraight = 0.40f;
    public float wActionRateTurn = 0.15f;

    [Tooltip("出现运动学翻转(Flip)惩罚，避免策略依赖>90°轮角翻转")]
    public float wFlipPenalty = 0.30f;

    [Header("Start pose")]
    public Vector3 startPos = new Vector3(1f, 0.25f, -1.233f);
    public Quaternion startRot = Quaternion.Euler(0f, 0f, 0f);

    [Header("Debug")]
    public bool enableDebugLog = false;
    public int debugLogEveryNFrames = 20;

    public float LastActionVx { get; private set; }
    public float LastActionOmega { get; private set; }

    private readonly float[] sensorRaw = new float[6];
    private readonly float[] sensorNorm = new float[6];

    private int lastSensorCacheFrame = -1;

    private float episodeTimer;
    private float filteredActionVx;
    private float filteredActionOmega;
    private float lastFilteredActionVx;
    private float lastFilteredActionOmega;
    private readonly float[] prevSteerAnglesRad = new float[4];

    public override void Initialize()
    {
        base.Initialize();
        if (rb == null) rb = GetComponent<Rigidbody>();
    }

    private void OnValidate()
    {
        constantForwardSpeed = Mathf.Max(0f, constantForwardSpeed);
        maxLateralSpeed = Mathf.Max(1e-4f, maxLateralSpeed);
        maxOmegaDeg = Mathf.Max(1e-4f, maxOmegaDeg);
        maxField = Mathf.Max(1e-4f, maxField);

        maxEpisodeTime = Mathf.Max(0.1f, maxEpisodeTime);
        derailThreshold = Mathf.Max(0f, derailThreshold);

        lrAbsDiffGood = Mathf.Clamp01(lrAbsDiffGood);
        lrAbsDiffBad = Mathf.Clamp01(lrAbsDiffBad);
        if (lrAbsDiffBad < lrAbsDiffGood)
        {
            float tmp = lrAbsDiffGood;
            lrAbsDiffGood = lrAbsDiffBad;
            lrAbsDiffBad = tmp;
        }

        lrMinSumNormForTrust = Mathf.Clamp01(lrMinSumNormForTrust);

        actionDeadzone = Mathf.Clamp01(actionDeadzone);
        actionFilterTau = Mathf.Max(1e-4f, actionFilterTau);
        maxActionRateVx = Mathf.Max(0f, maxActionRateVx);
        maxActionRateOmega = Mathf.Max(0f, maxActionRateOmega);

        straightHoldStartQ = Mathf.Clamp01(straightHoldStartQ);
        straightHoldFullQ = Mathf.Clamp01(straightHoldFullQ);
        if (straightHoldFullQ < straightHoldStartQ)
        {
            float tmp = straightHoldStartQ;
            straightHoldStartQ = straightHoldFullQ;
            straightHoldFullQ = tmp;
        }

        straightHoldStrength = Mathf.Clamp01(straightHoldStrength);
        debugLogEveryNFrames = Mathf.Max(1, debugLogEveryNFrames);
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

        episodeTimer = 0f;
        filteredActionVx = 0f;
        filteredActionOmega = 0f;
        lastFilteredActionVx = 0f;
        lastFilteredActionOmega = 0f;
        LastActionVx = 0f;
        LastActionOmega = 0f;
        for (int i = 0; i < prevSteerAnglesRad.Length; i++) prevSteerAnglesRad[i] = 0f;

        if (myCarMotion != null) myCarMotion.SetControl(constantForwardSpeed, 0f, 0f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        UpdateSensorCache();

        // 1-6: 六个传感器归一化强度
        for (int i = 0; i < sensorNorm.Length; i++) sensor.AddObservation(sensorNorm[i]);

        // 7-9: 运动状态（归一化）
        Vector3 velWorld = rb != null ? rb.linearVelocity : Vector3.zero;
        Vector3 localVel = transform.InverseTransformDirection(velWorld);
        sensor.AddObservation(Mathf.Clamp(localVel.z / Mathf.Max(1e-4f, constantForwardSpeed), -2f, 2f));
        sensor.AddObservation(Mathf.Clamp(localVel.x / Mathf.Max(1e-4f, maxLateralSpeed), -2f, 2f));

        float maxOmegaRad = maxOmegaDeg * Mathf.Deg2Rad;
        float yawRate = rb != null ? rb.angularVelocity.y : 0f;
        sensor.AddObservation(Mathf.Clamp(yawRate / Mathf.Max(1e-4f, maxOmegaRad), -2f, 2f));

        // 10-12: 派生跟踪特征（连续，不区分转弯/直线模式）
        ComputeTrackingScores(out float alignment, out float centerNorm, out float trackQuality);
        sensor.AddObservation(alignment);
        sensor.AddObservation(centerNorm);
        sensor.AddObservation(trackQuality);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float dt = Time.fixedDeltaTime;

        UpdateSensorCache();
        ComputeTrackingScores(out float alignment, out float centerNorm, out float trackQuality);

        float fcRaw = sensorRaw[1];
        float rcRaw = sensorRaw[4];

        // ========== 终止：中心离线 ==========
        if (fcRaw < derailThreshold || rcRaw < derailThreshold)
        {
            AddReward(-5f);
            if (enableDebugLog)
                Debug.Log($"Episode Ended: derailment. fc={fcRaw:F3}, rc={rcRaw:F3}");
            EndEpisode();
            return;
        }

        // ========== 输入动作（归一化） ==========
        float rawVx = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float rawOmega = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        // 动作死区（直接抑制微小抖动源）
        if (Mathf.Abs(rawVx) < actionDeadzone) rawVx = 0f;
        if (Mathf.Abs(rawOmega) < actionDeadzone) rawOmega = 0f;

        // 直线保持（连续门控：trackQuality越高，越逼近vx=0、omega=0）
        float hold = 0f;
        if (useStraightHold)
        {
            float t = Smooth01((trackQuality - straightHoldStartQ) / Mathf.Max(1e-4f, straightHoldFullQ - straightHoldStartQ));
            hold = straightHoldStrength * SmoothStep01(t);
        }
        float cmdVx = rawVx * (1f - hold);
        float cmdOmega = rawOmega * (1f - hold);

        // 动作滤波（低通）+ 速率限制（抑制高频反复修正）
        float alpha = 1f - Mathf.Exp(-dt / Mathf.Max(1e-4f, actionFilterTau));
        float targetVx = Mathf.Lerp(filteredActionVx, cmdVx, alpha);
        float targetOmega = Mathf.Lerp(filteredActionOmega, cmdOmega, alpha);

        float maxStepVx = Mathf.Max(0f, maxActionRateVx) * dt;
        float maxStepOmega = Mathf.Max(0f, maxActionRateOmega) * dt;
        filteredActionVx = Mathf.MoveTowards(filteredActionVx, targetVx, maxStepVx);
        filteredActionOmega = Mathf.MoveTowards(filteredActionOmega, targetOmega, maxStepOmega);

        // 下发控制：固定Vz，仅输出vx/omega
        float vz = constantForwardSpeed;
        float vx = filteredActionVx * maxLateralSpeed;
        float omega = filteredActionOmega * maxOmegaDeg * Mathf.Deg2Rad;
        if (myCarMotion != null) myCarMotion.SetControl(vz, vx, omega);

        // ========== 奖励/惩罚 ==========
        Vector3 velWorld = rb != null ? rb.linearVelocity : Vector3.zero;
        Vector3 localVel = transform.InverseTransformDirection(velWorld);
        float forwardSpeed = Vector3.Dot(velWorld, transform.forward);
        float forwardNorm = Mathf.Clamp01(forwardSpeed / Mathf.Max(1e-4f, constantForwardSpeed));

        // 直线性：trackQuality高→更像直线跟踪→更严苛地抑制侧滑/摆头/摆轮
        float straightness = SmoothStep01(Smooth01((trackQuality - straightHoldStartQ) / Mathf.Max(1e-4f, 1f - straightHoldStartQ)));

        float wLatVel = Mathf.Lerp(wLatVelTurn, wLatVelStraight, straightness);
        float wYawRate = Mathf.Lerp(wYawRateTurn, wYawRateStraight, straightness);
        float wSteerAbs = Mathf.Lerp(wSteerAbsTurn, wSteerAbsStraight, straightness);
        float wSteerRate = Mathf.Lerp(wSteerRateTurn, wSteerRateStraight, straightness);
        float wActMag = Mathf.Lerp(wActionMagTurn, wActionMagStraight, straightness);
        float wActRate = Mathf.Lerp(wActionRateTurn, wActionRateStraight, straightness);

        float latVelNorm = Mathf.Clamp(localVel.x / Mathf.Max(1e-4f, maxLateralSpeed), -2f, 2f);

        float maxOmegaRad = maxOmegaDeg * Mathf.Deg2Rad;
        float yawRate = rb != null ? rb.angularVelocity.y : 0f;
        float yawRateNorm = Mathf.Clamp(yawRate / Mathf.Max(1e-4f, maxOmegaRad), -2f, 2f);

        float rewardPerSec = 0f;
        rewardPerSec += wTrackQuality * trackQuality;
        rewardPerSec += wForward * forwardNorm;

        rewardPerSec -= wLatVel * (latVelNorm * latVelNorm);
        rewardPerSec -= wYawRate * (yawRateNorm * yawRateNorm);

        // 轮子摆动惩罚：幅度 + 变化率
        float steerAbsNormAvg = 0f;
        float steerRateNormAvg = 0f;
        if (myCarMotion != null && myCarMotion.steerAngles != null && myCarMotion.steerAngles.Length >= 4)
        {
            for (int i = 0; i < 4; i++)
            {
                float steerRad = myCarMotion.steerAngles[i];
                steerAbsNormAvg += Mathf.Clamp01(Mathf.Abs(steerRad) * Mathf.Rad2Deg / 90f);

                float prevRad = prevSteerAnglesRad[i];
                float dDeg = Mathf.Abs(Mathf.DeltaAngle(prevRad * Mathf.Rad2Deg, steerRad * Mathf.Rad2Deg));
                float rateDegPerSec = dDeg / Mathf.Max(1e-4f, dt);
                steerRateNormAvg += Mathf.Clamp01(rateDegPerSec / 360f);

                prevSteerAnglesRad[i] = steerRad;
            }
            steerAbsNormAvg /= 4f;
            steerRateNormAvg /= 4f;
        }
        rewardPerSec -= wSteerAbs * (steerAbsNormAvg * steerAbsNormAvg);
        rewardPerSec -= wSteerRate * (steerRateNormAvg * steerRateNormAvg);

        // 动作幅度/变化率惩罚（抑制策略用抖动取巧）
        float actMag = (filteredActionVx * filteredActionVx) + (filteredActionOmega * filteredActionOmega);
        float dAct = Mathf.Abs(filteredActionVx - lastFilteredActionVx) + Mathf.Abs(filteredActionOmega - lastFilteredActionOmega);
        rewardPerSec -= wActMag * actMag;
        rewardPerSec -= wActRate * (dAct * dAct);

        // Flip惩罚
        if (myCarMotion != null && myCarMotion.flipOccurred) rewardPerSec -= wFlipPenalty;

        AddReward(rewardPerSec * dt);

        lastFilteredActionVx = filteredActionVx;
        lastFilteredActionOmega = filteredActionOmega;
        LastActionVx = filteredActionVx;
        LastActionOmega = filteredActionOmega;

        // Debug（节流）
        if (enableDebugLog && (debugLogEveryNFrames <= 1 || Time.frameCount % debugLogEveryNFrames == 0))
        {
            Debug.Log($"Q={trackQuality:F3} (align={alignment:F3}, center={centerNorm:F3}), hold={hold:F2}, act=({filteredActionVx:F2},{filteredActionOmega:F2}), localVx={localVel.x:F3}, yaw={yawRate:F3}");
        }

        // 超时
        episodeTimer += dt;
        if (episodeTimer >= maxEpisodeTime)
        {
            if (enableDebugLog) Debug.Log($"Episode Ended: timeout. t={episodeTimer:F2}s");
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // 不提供手动控制
    }

    private void UpdateSensorCache()
    {
        // ML-Agents常见调用顺序：CollectObservations 和 OnActionReceived 可能在同一帧连续调用
        // 这里做一次帧级缓存，避免重复读取磁场（不改变语义）
        int frame = Time.frameCount;
        if (frame == lastSensorCacheFrame) return;
        lastSensorCacheFrame = frame;

        float invMax = 1f / Mathf.Max(1e-4f, maxField);
        for (int i = 0; i < sensorRaw.Length; i++)
        {
            float mag = 0f;
            if (tape != null && sensors != null && i < sensors.Length && sensors[i] != null)
            {
                mag = tape.GetMagneticField(sensors[i].position).magnitude;
            }
            sensorRaw[i] = mag;
            sensorNorm[i] = Mathf.Clamp01(mag * invMax);
        }
    }

    private void ComputeTrackingScores(out float alignment, out float centerNorm, out float trackQuality)
    {
        float fl = sensorRaw[0];
        float fc = sensorRaw[1];
        float fr = sensorRaw[2];
        float rl = sensorRaw[3];
        float rc = sensorRaw[4];
        float rr = sensorRaw[5];

        float invMax = 1f / Mathf.Max(1e-4f, maxField);

        float frontAbsDiffNorm = Mathf.Abs(fl - fr) * invMax;
        float rearAbsDiffNorm = Mathf.Abs(rl - rr) * invMax;

        float frontAlign = ScoreSmall(frontAbsDiffNorm, lrAbsDiffGood, lrAbsDiffBad);
        float rearAlign = ScoreSmall(rearAbsDiffNorm, lrAbsDiffGood, lrAbsDiffBad);
        float rawAlign = Mathf.Min(frontAlign, rearAlign);

        float frontSumNorm = (Mathf.Abs(fl) + Mathf.Abs(fr)) * (0.5f * invMax);
        float rearSumNorm = (Mathf.Abs(rl) + Mathf.Abs(rr)) * (0.5f * invMax);
        float lrSumNorm = Mathf.Min(frontSumNorm, rearSumNorm);
        float trust = SmoothStep01(Smooth01((lrSumNorm - lrMinSumNormForTrust) / Mathf.Max(1e-4f, 1f - lrMinSumNormForTrust)));

        alignment = rawAlign * trust;
        centerNorm = Mathf.Clamp01(Mathf.Min(fc, rc) * invMax);
        trackQuality = alignment * centerNorm;
    }

    private static float ScoreSmall(float x, float good, float bad)
    {
        // x<=good => 1; x>=bad => 0
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