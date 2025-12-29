using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using UnityEngine;
using System.Collections.Generic;
using System.Collections;

public class StackGenerator : MonoBehaviour
{
    [Header("粒子设置")]
    public GameObject particlePrefab;
    public float particleRadius = 0.1f;
    public float gap = 0.02f; // 粒子之间的间隙

    [Header("圆柱参数")]
    public float stackRadius = 1.0f;
    public float stackHeight = 2.0f;

    [Header("生成选项")]
    public bool generateOnStart = true;
    public Material particleMaterial;

    [Header("填充参数")]
    [Range(0.1f, 1.0f)]
    public float fillDensity = 0.8f; // 填充密度（1.0为完全填充）
    public bool fillInterior = true; // 是否填充内部

    [Header("GUI设置")]
    public bool showGUI = true;
    public Vector2 guiPosition = new Vector2(10, 10);

    private List<GameObject> generatedParticles = new List<GameObject>();
    private bool isGenerating = false;

    void Start()
    {
        if (generateOnStart)
        {
            StartCoroutine(GenerateStack());
        }
    }

    void OnGUI()
    {
        if (!showGUI) return;

        // 生成按钮
        GUI.enabled = !isGenerating;
        if (GUILayout.Button(isGenerating ? "生成中..." : "生成堆栈", GUILayout.Height(40)))
        {
            StartCoroutine(GenerateStack());
        }
        GUI.enabled = true;

        // 显示粒子数量
        GUILayout.Label($"当前粒子数: {generatedParticles.Count}");
    }

    IEnumerator GenerateStack()
    {
        if (isGenerating) yield break;

        isGenerating = true;
        ClearStack();

        float particleDiameter = particleRadius * 2f;
        float effectiveDiameter = particleDiameter + gap; // 包含间隙的有效直径

        // 计算层数
        float layerHeight = Mathf.Sqrt(3f) * effectiveDiameter / 2f; // 六角密堆积的层高
        int layers = Mathf.FloorToInt(stackHeight / layerHeight);

        int totalParticles = 0;

        for (int layer = 0; layer < layers; layer++)
        {
            float yPos = layer * layerHeight;

            // 判断是否需要生成这一层（根据填充密度）
            if (Random.value > fillDensity) continue;

            // 交替层的偏移量，实现密堆积
            bool isEvenLayer = layer % 2 == 0;
            float horizontalOffset = isEvenLayer ? 0f : effectiveDiameter / 2f;

            // 计算这一层的半径范围
            float maxRadius = stackRadius - particleRadius;

            if (fillInterior)
            {
                // 填充内部：从内向外生成同心圆环
                int ringCount = Mathf.FloorToInt(maxRadius / effectiveDiameter);

                for (int ring = 0; ring < ringCount; ring++)
                {
                    // 当前环的半径
                    float currentRadius = (ring + 0.5f) * effectiveDiameter;

                    // 如果当前环的半径超过了最大半径，跳过
                    if (currentRadius > maxRadius) break;

                    // 计算这一环可以容纳多少粒子
                    float circumference = 2f * Mathf.PI * currentRadius;
                    int particlesInRing = Mathf.FloorToInt(circumference / effectiveDiameter);

                    // 如果环太小，放不下粒子，跳过
                    if (particlesInRing < 3) continue;

                    for (int i = 0; i < particlesInRing; i++)
                    {
                        float angle = (i * 2f * Mathf.PI / particlesInRing);

                        // 添加水平偏移以实现密堆积
                        float xPos = currentRadius * Mathf.Cos(angle) + horizontalOffset;
                        float zPos = currentRadius * Mathf.Sin(angle) + horizontalOffset;

                        // 检查位置是否在圆柱内
                        float distanceFromCenter = Mathf.Sqrt(xPos * xPos + zPos * zPos);
                        if (distanceFromCenter > maxRadius) continue;

                        Vector3 position = new Vector3(xPos, yPos, zPos);
                        CreateParticle(position);
                        totalParticles++;

                        // 每生成100个粒子等待一帧，避免卡顿
                        if (totalParticles % 100 == 0)
                        {
                            yield return null;
                        }
                    }
                }
            }
            else
            {
                // 只生成外层
                float currentRadius = maxRadius;
                float circumference = 2f * Mathf.PI * currentRadius;
                int particlesInRing = Mathf.FloorToInt(circumference / effectiveDiameter);

                for (int i = 0; i < particlesInRing; i++)
                {
                    float angle = (i * 2f * Mathf.PI / particlesInRing);
                    float xPos = currentRadius * Mathf.Cos(angle) + horizontalOffset;
                    float zPos = currentRadius * Mathf.Sin(angle) + horizontalOffset;

                    Vector3 position = new Vector3(xPos, yPos, zPos);
                    CreateParticle(position);
                    totalParticles++;
                }

                // 每层之间等待一帧
                yield return null;
            }
        }

        // 在中心位置添加一个粒子（如果填充内部）
        if (fillInterior)
        {
            CreateParticle(Vector3.zero);
            totalParticles++;
        }

        Debug.Log($"生成了 {generatedParticles.Count} 个粒子，共 {layers} 层");

        foreach(var item in generatedParticles)
        {
            item.SetActive(true);
        }
        isGenerating = false;
    }

    void CreateParticle(Vector3 position)
    {
        GameObject particle = Instantiate(particlePrefab, transform.position + position, Quaternion.identity, transform);
        particle.name = "Particle";
        particle.transform.localScale = Vector3.one * (particleRadius * 2f);
        //particle.SetActive(true);
        // 应用材质
        if (particleMaterial != null)
        {
            Renderer renderer = particle.GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.material = particleMaterial;
            }
        }

        generatedParticles.Add(particle);
    }

    [ContextMenu("清除堆栈")]
    public void ClearStack()
    {
        //for (int i = generatedParticles.Count - 1; i >= 0; i--)
        //{
        //    if (generatedParticles[i] != null)
        //    {
        //        if (Application.isPlaying)
        //        {
        //            Destroy(generatedParticles[i]);
        //        }
        //        else
        //        {
        //            DestroyImmediate(generatedParticles[i]);
        //        }
        //    }
        //}
        //generatedParticles.Clear(); 
    }

    void OnDrawGizmosSelected()
    {
        // 绘制圆柱轮廓
        Gizmos.color = Color.cyan;
        Gizmos.matrix = transform.localToWorldMatrix;

        // 绘制底部圆
        DrawCircle(Vector3.zero, stackRadius);

        // 绘制顶部圆
        DrawCircle(Vector3.up * stackHeight, stackRadius);

        // 绘制侧面
        DrawCylinderOutline(stackRadius, stackHeight);

        Gizmos.matrix = Matrix4x4.identity;
    }

    void DrawCircle(Vector3 center, float radius)
    {
        int segments = 32;
        float angleStep = 2f * Mathf.PI / segments;

        Vector3 prevPoint = center + new Vector3(radius, 0, 0);
        for (int i = 1; i <= segments; i++)
        {
            float angle = i * angleStep;
            Vector3 nextPoint = center + new Vector3(
                Mathf.Cos(angle) * radius,
                0,
                Mathf.Sin(angle) * radius
            );

            Gizmos.DrawLine(prevPoint, nextPoint);
            prevPoint = nextPoint;
        }
    }

    void DrawCylinderOutline(float radius, float height)
    {
        int segments = 8;
        float angleStep = 2f * Mathf.PI / segments;

        for (int i = 0; i < segments; i++)
        {
            float angle = i * angleStep;
            Vector3 bottomPoint = new Vector3(
                Mathf.Cos(angle) * radius,
                0,
                Mathf.Sin(angle) * radius
            );
            Vector3 topPoint = bottomPoint + Vector3.up * height;

            Gizmos.DrawLine(bottomPoint, topPoint);
        }
    }

    void OnValidate()
    {
        // 确保参数合法
        particleRadius = Mathf.Max(0.01f, particleRadius);
        gap = Mathf.Max(0f, gap);
        stackRadius = Mathf.Max(particleRadius * 2f, stackRadius);
        stackHeight = Mathf.Max(particleRadius * 2f, stackHeight);
        fillDensity = Mathf.Clamp01(fillDensity);
    }

    // 计算预期粒子数量
    public int CalculateExpectedParticleCount()
    {
        float effectiveDiameter = (particleRadius * 2f) + gap;
        float layerHeight = Mathf.Sqrt(3f) * effectiveDiameter / 2f;

        int layers = Mathf.FloorToInt(stackHeight / layerHeight);
        int expectedLayers = Mathf.FloorToInt(layers * fillDensity);

        if (!fillInterior)
        {
            // 只计算外层
            float maxRadius = stackRadius - particleRadius;
            float circumference = 2f * Mathf.PI * maxRadius;
            int particlesInRing = Mathf.FloorToInt(circumference / effectiveDiameter);
            return particlesInRing * expectedLayers + (fillInterior ? 1 : 0);
        }
        else
        {
            // 计算所有环
            int totalParticles = 0;
            float maxRadius = stackRadius - particleRadius;
            int ringCount = Mathf.FloorToInt(maxRadius / effectiveDiameter);

            for (int ring = 0; ring < ringCount; ring++)
            {
                float currentRadius = (ring + 0.5f) * effectiveDiameter;
                if (currentRadius > maxRadius) break;

                float circumference = 2f * Mathf.PI * currentRadius;
                int particlesInRing = Mathf.FloorToInt(circumference / effectiveDiameter);
                if (particlesInRing >= 3)
                {
                    totalParticles += particlesInRing * expectedLayers;
                }
            }

            return totalParticles + (fillInterior ? 1 : 0); // 加上中心粒子
        }
    }
}
