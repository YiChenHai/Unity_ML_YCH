using UnityEngine;

public class FPSDisplay : MonoBehaviour
{
    private float deltaTime = 0.0f;
    private float fps = 0.0f;
    private float updateInterval = 0.5f; // 每0.5秒更新一次
    private float accum = 0.0f;
    private int frames = 0;
    private float timeLeft;

    void Start()
    {
        timeLeft = updateInterval;
    }

    void Update()
    {
        timeLeft -= Time.deltaTime;
        accum += Time.timeScale / Time.deltaTime;
        frames++;

        if (timeLeft <= 0.0f)
        {
            fps = accum / frames;
            timeLeft = updateInterval;
            accum = 0.0f;
            frames = 0;
        }
    }

    void OnGUI()
    {
        int w = Screen.width, h = Screen.height;

        GUIStyle style = new GUIStyle();
        Rect rect = new Rect(w - 150, 10, 140, 40);
        style.alignment = TextAnchor.UpperRight;
        style.fontSize = 24;
        style.normal.textColor = fps < 30 ? Color.red : (fps < 60 ? Color.yellow : Color.green);
        style.fontStyle = FontStyle.Bold;

        string text = string.Format("{0:0.} FPS", fps);
        GUI.Label(rect, text, style);

        // 显示Time.timeScale
        GUIStyle scaleStyle = new GUIStyle(style);
        scaleStyle.fontSize = 16;
        scaleStyle.normal.textColor = Color.cyan;
        Rect scaleRect = new Rect(w - 150, 50, 140, 30);
        GUI.Label(scaleRect, $"TimeScale: {Time.timeScale:F1}x", scaleStyle);
    }
}
