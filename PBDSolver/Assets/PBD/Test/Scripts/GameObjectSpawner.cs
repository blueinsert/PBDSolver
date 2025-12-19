using bluebean.Physics.PBD;
using UnityEngine;

public class GameObjectSpawner : MonoBehaviour
{
    [Header("生成设置")]
    [Tooltip("将要实例化的Prefab")]
    public GameObject prefabToSpawn;

    [Tooltip("生成位置的偏移量（相对于当前对象位置）")]
    public Vector3 spawnOffset = Vector3.zero;

    [Tooltip("是否使用当前对象的旋转")]
    public bool useCurrentRotation = true;

    [Tooltip("生成的物体是否作为子对象")]
    public bool asChild = true;

    [Tooltip("每秒最大生成次数（防止连点）")]
    public float maxSpawnRate = 10f;

    [Header("调试")]
    [Tooltip("生成时是否显示调试信息")]
    public bool showDebugLog = true;

    private float lastSpawnTime = 0f;

    private int m_id = 1;

    void Update()
    {
        // 检测空格键按下
        if (Input.GetKeyDown(KeyCode.Space))
        {
            TrySpawnPrefab();
        }

        // 可选：按住空格连续生成
        // if (Input.GetKey(KeyCode.Space))
        // {
        //     TrySpawnPrefab();
        // }
    }

    void TrySpawnPrefab()
    {
        // 检查Prefab是否已赋值
        if (prefabToSpawn == null)
        {
            Debug.LogWarning("请先为GameObjectSpawner的prefabToSpawn变量赋值！");
            return;
        }

        // 检查生成频率限制
        if (Time.time - lastSpawnTime < 1f / maxSpawnRate)
        {
            return;
        }

        // 实例化Prefab
        SpawnPrefab();
    }

    void SpawnPrefab()
    {
        // 计算生成位置和旋转
        Vector3 spawnPosition = transform.position + spawnOffset;
        Quaternion spawnRotation = useCurrentRotation ? transform.rotation : prefabToSpawn.transform.rotation;

        // 实例化对象
        GameObject spawnedObject = Instantiate(prefabToSpawn, spawnPosition, spawnRotation);
        var actor = spawnedObject.GetComponent<PBDActor>();
        if (actor != null)
        {
            actor.m_actorId = m_id++;
        }
        // 设置父对象
        if (asChild)
        {
            spawnedObject.transform.SetParent(transform);
        }

        spawnedObject.gameObject.SetActive(true);
        // 记录生成时间
        lastSpawnTime = Time.time;

        // 调试信息
        if (showDebugLog)
        {
            Debug.Log($"已生成Prefab: {prefabToSpawn.name}，位置: {spawnPosition}，父对象: {transform.name}");
        }
    }

    // 可选：提供公共方法供其他脚本调用
    public void Spawn()
    {
        TrySpawnPrefab();
    }

    // 在场景视图中显示生成区域（调试用）
    void OnDrawGizmosSelected()
    {
        if (enabled)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(transform.position + spawnOffset, 0.5f);
            Gizmos.DrawIcon(transform.position + spawnOffset, "Prefab Icon", true);
        }
    }
}