using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 模拟磁传感器 Magnetic Sensor
/// </summary>
public class MagneticSensor : MonoBehaviour
{
    [Header("磁条引用")]
    public MagneticTape magneticTape;  // 绑定磁条对象（而非磁导线）

    [Tooltip("当前传感器测得的磁场强度 (单位化值)")]
    public float fieldStrength;  // 用于存储磁场的强度（标量）

    [Tooltip("当前传感器测得的磁场方向")]
    public Vector3 fieldDirection;  // 用于存储磁场的方向（矢量）

    [Tooltip("是否显示磁场方向箭头")]
    public bool showDirection = false;

    void Update()
    {
        if (magneticTape != null)
        {
            // 调用磁条脚本的方法来获取磁场矢量
            Vector3 magneticField = magneticTape.GetMagneticField(transform.position);
            fieldStrength = magneticField.magnitude;  // 获取磁场强度（标量）
            fieldDirection = magneticField.normalized;  // 获取磁场方向（单位向量）
        }
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(transform.position, 0.02f);

        // 可选：可视化磁场方向
        if (showDirection && magneticTape != null)
        {
            // 使用 GetMagneticField 来获取磁场矢量
            Vector3 magneticField = magneticTape.GetMagneticField(transform.position);
            float visualScale = 10f; // 可调大以看见箭头
            if (magneticField.magnitude > 1e-8f)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(transform.position, transform.position + magneticField.normalized * visualScale * magneticField.magnitude);
            }
        }
    }
}
