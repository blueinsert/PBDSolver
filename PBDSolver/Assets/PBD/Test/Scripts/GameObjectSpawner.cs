using bluebean.Physics.PBD;
using UnityEngine;

public class GameObjectSpawner : MonoBehaviour
{
    [Header("生成设置")]
    [Tooltip("将要实例化的Prefab")]
    public GameObject prefabToSpawn;
    [Header("触发按钮")]
    public KeyCode triggerKey;
    [Tooltip("生成位置的偏移量（相对于当前对象位置）")]
    public Vector3 spawnOffset = Vector3.zero;
    public bool isRangeSpawn = false;
    public Vector3 spawnRange = Vector3.zero;
    [Header("初始速度")]
    public Vector3 initVel = Vector3.zero;
    [Header("初始速度扰动")]
    public Vector3 initVelPerturb = Vector3.zero;
    [Header("生成物体父节点")]
    public GameObject m_root = null;
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
        if (Input.GetKeyDown(this.triggerKey))
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
        if (isRangeSpawn)
        {
            float xoffset = spawnRange.x * (Random.Range(1, 100) / 100f - 0.5f);
            float yoffset = spawnRange.y * (Random.Range(1, 100) / 100f - 0.5f);
            float zoffset = spawnRange.z * (Random.Range(1, 100) / 100f - 0.5f);
            spawnPosition += new Vector3(xoffset, yoffset, zoffset);
        }
        Quaternion spawnRotation = Quaternion.identity;

        // 实例化对象
        GameObject spawnedObject = Instantiate(prefabToSpawn, spawnPosition, spawnRotation);
        var actor = spawnedObject.GetComponent<PBDActor>();
        if (actor != null)
        {
            actor.m_actorId = m_id++;
            actor.m_initVel = new Vector3(this.initVel.x + this.initVelPerturb.x*Random.Range(-1.0f,1.0f),
                this.initVel.y + this.initVelPerturb.y * Random.Range(-1.0f, 1.0f),
                this.initVel.z + this.initVelPerturb.z * Random.Range(-1.0f, 1.0f));
        }else
        {
            var rigid = spawnedObject.GetComponent<Rigidbody>();
            if(rigid != null)
            {
                rigid.velocity = new Vector3(this.initVel.x + this.initVelPerturb.x * Random.Range(-1.0f, 1.0f),
                this.initVel.y + this.initVelPerturb.y * Random.Range(-1.0f, 1.0f),
                this.initVel.z + this.initVelPerturb.z * Random.Range(-1.0f, 1.0f));
            }
        }


        spawnedObject.transform.SetParent(this.m_root.transform);

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