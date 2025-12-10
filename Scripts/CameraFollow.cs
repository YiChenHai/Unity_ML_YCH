using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    [Header("Target")]
    public Transform target; // 要跟随的车辆（MyCarAgent GameObject）

    [Header("Offset")]
    public Vector3 positionOffset = new Vector3(0f, 2f, -4f); // 相对于车的位置偏移
    public Vector3 rotationOffset = Vector3.zero; // 旋转偏移

    [Header("Smoothing")]
    public float positionSmoothTime = 0.3f; // 位置平滑系数（越小越快跟随）
    public float rotationSmoothTime = 0.3f; // 旋转平滑系数

    [Header("Look At")]
    public bool lookAtTarget = true; // 是否始终看向目标
    public Vector3 lookAtOffset = new Vector3(0f, 0.5f, 0f); // 看向目标时的偏移

    private Vector3 velocity = Vector3.zero; // 用于平滑移动
    private Vector3 currentRotation = Vector3.zero; // 当前旋转欧拉角
    private Vector3 rotationVelocity = Vector3.zero; // 用于平滑旋转

    void LateUpdate()
    {
        if (target == null) return;

        // 计算目标位置（相对于车身坐标系）
        Vector3 targetPosition = target.position + target.TransformDirection(positionOffset);

        // 平滑移动摄像头到目标位置
        transform.position = Vector3.SmoothDamp(
            transform.position,
            targetPosition,
            ref velocity,
            positionSmoothTime
        );

        // 设置摄像头朝向
        if (lookAtTarget)
        {
            // 看向目标的位置
            Vector3 lookTarget = target.position + lookAtOffset;
            Vector3 directionToTarget = lookTarget - transform.position;
            
            if (directionToTarget.sqrMagnitude > 0.001f)
            {
                Quaternion targetRotation = Quaternion.LookRotation(directionToTarget);
                transform.rotation = Quaternion.Slerp(
                    transform.rotation,
                    targetRotation,
                    Time.deltaTime / (rotationSmoothTime + 0.001f)
                );
            }
        }
        else
        {
            // 跟随车辆的旋转加上偏移
            Quaternion targetRotation = target.rotation * Quaternion.Euler(rotationOffset);
            transform.rotation = Quaternion.Slerp(
                transform.rotation,
                targetRotation,
                Time.deltaTime / (rotationSmoothTime + 0.001f)
            );
        }
    }
}
