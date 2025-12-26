using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

/// <summary>
/// 磁导航训练数据收集器：记录观测和动作用于决策树训练
/// 将此脚本挂载到与MyCarAgent相同的GameObject上
/// </summary>
public class MyCar_DataCollector : MonoBehaviour
{
    [Header("References")]
    public MyCarAgent agent;
    public MagneticTape tape;
    public Transform[] sensors = new Transform[6];
    public Rigidbody rb;
    
    [Header("Data Collection Settings")]
    [Tooltip("是否启用数据收集")]
    public bool enableCollection = true;
    [Tooltip("CSV文件保存路径（相对Application.dataPath）")]
    public string dataFilePath = "../TrainingData/magnetic_tracking_data.csv";
    [Tooltip("每N帧记录一次（避免数据量过大）")]
    public int sampleInterval = 1;
    [Tooltip("最大记录行数（0=无限制）")]
    public int maxRecords = 100000;
    
    private StreamWriter dataWriter;
    private int frameCounter = 0;
    private int recordCount = 0;
    private string fullPath;
    private bool isInitialized = false;
    
    void Start()
    {
        // 自动查找引用
        if (agent == null) agent = GetComponent<MyCarAgent>();
        if (rb == null) rb = GetComponent<Rigidbody>();
        if (tape == null) tape = agent?.tape;
        if (sensors == null || sensors.Length == 0) sensors = agent?.sensors;
        
        if (!enableCollection) return;
        
        // 构建完整路径
        fullPath = Path.Combine(Application.dataPath, dataFilePath);
        string directory = Path.GetDirectoryName(fullPath);
        
        // 确保目录存在
        if (!Directory.Exists(directory))
        {
            Directory.CreateDirectory(directory);
        }
        
        // 打开文件（追加模式）
        try
        {
            bool fileExists = File.Exists(fullPath);
            dataWriter = new StreamWriter(fullPath, true);
            
            // 如果是新文件，写入表头
            if (!fileExists)
            {
                WriteHeader();
            }
            
            isInitialized = true;
            Debug.Log($"<color=green>Data Collector:</color> Initialized. Saving to: {fullPath}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"<color=red>Data Collector:</color> Failed to initialize: {e.Message}");
            enableCollection = false;
        }
    }
    
    void WriteHeader()
    {
        dataWriter.WriteLine(
            "sens0,sens1,sens2,sens3,sens4,sens5," +
            "vel_z_norm,vel_x_norm,omega_norm," +
            "front_diff_smooth,rear_diff_smooth,turn_mode," +
            "action_vx,action_omega"
        );
    }
    
    void FixedUpdate()
    {
        if (!enableCollection || !isInitialized || agent == null) return;
        
        // 检查是否达到最大记录数
        if (maxRecords > 0 && recordCount >= maxRecords)
        {
            if (enableCollection)
            {
                Debug.Log($"<color=yellow>Data Collector:</color> Reached max records ({maxRecords}). Stopping collection.");
                enableCollection = false;
                CloseFile();
            }
            return;
        }
        
        // 采样间隔控制
        frameCounter++;
        if (frameCounter % sampleInterval != 0) return;
        
        // 收集数据
        CollectData();
    }
    
    void CollectData()
    {
        // ========== 收集观测数据 ==========
        float[] sensorValues = new float[6];
        for (int i = 0; i < 6; i++)
        {
            if (i < sensors.Length && sensors[i] != null && tape != null)
            {
                Vector3 mag = tape.GetMagneticField(sensors[i].position);
                sensorValues[i] = Mathf.Clamp01(mag.magnitude / Mathf.Max(1e-9f, agent.maxField));
            }
        }
        
        // 运动状态
        Vector3 localVel = transform.InverseTransformDirection(rb != null ? rb.linearVelocity : Vector3.zero);
        float vel_z_norm = localVel.z / Mathf.Max(0.001f, agent.constantForwardSpeed);
        float vel_x_norm = localVel.x / Mathf.Max(0.001f, agent.maxLateralSpeed);
        
        float angularVel = rb != null ? rb.angularVelocity.y : 0f;
        float maxOmegaRad = agent.maxOmegaDeg * Mathf.Deg2Rad;
        float omega_norm = Mathf.Clamp(angularVel / maxOmegaRad, -1f, 1f);
        
        // 转弯判定信号
        float front_diff = agent.FrontDiffSmoothed;
        float rear_diff = agent.RearDiffSmoothed;
        float turn_mode = agent.IsInTurnMode ? 1f : 0f;
        
        // ========== 获取动作输出 ==========
        float action_vx = agent.LastActionVx;
        float action_omega = agent.LastActionOmega;
        
        // ========== 写入CSV ==========
        string line = string.Format(
            "{0:F6},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6}," +
            "{6:F6},{7:F6},{8:F6}," +
            "{9:F6},{10:F6},{11:F6}," +
            "{12:F6},{13:F6}",
            sensorValues[0], sensorValues[1], sensorValues[2],
            sensorValues[3], sensorValues[4], sensorValues[5],
            vel_z_norm, vel_x_norm, omega_norm,
            front_diff, rear_diff, turn_mode,
            action_vx, action_omega
        );
        
        dataWriter.WriteLine(line);
        recordCount++;
        
        // 定期刷新缓冲区并显示进度
        if (recordCount % 1000 == 0)
        {
            dataWriter.Flush();
            Debug.Log($"<color=cyan>Data Collector:</color> Progress: {recordCount} records");
        }
    }
    
    void CloseFile()
    {
        if (dataWriter != null)
        {
            dataWriter.Flush();
            dataWriter.Close();
            dataWriter = null;
            Debug.Log($"<color=green>Data Collector:</color> Stopped. Total records: {recordCount}");
        }
    }
    
    void OnDestroy()
    {
        CloseFile();
    }
    
    void OnApplicationQuit()
    {
        CloseFile();
    }
    
    // Inspector按钮：手动停止收集
    [ContextMenu("Stop Collection")]
    public void StopCollection()
    {
        enableCollection = false;
        CloseFile();
    }
    
    // Inspector按钮：清空并重新开始
    [ContextMenu("Reset and Restart")]
    public void ResetAndRestart()
    {
        CloseFile();
        recordCount = 0;
        frameCounter = 0;
        
        if (File.Exists(fullPath))
        {
            File.Delete(fullPath);
            Debug.Log($"<color=yellow>Data Collector:</color> Deleted old file: {fullPath}");
        }
        
        Start();
    }
    
    // Inspector按钮：查看当前统计
    [ContextMenu("Show Stats")]
    public void ShowStats()
    {
        Debug.Log($"<color=cyan>Data Collector Stats:</color>\n" +
                  $"Enabled: {enableCollection}\n" +
                  $"Records: {recordCount}\n" +
                  $"File: {fullPath}\n" +
                  $"Sample Interval: {sampleInterval}\n" +
                  $"Max Records: {maxRecords}");
    }
}
