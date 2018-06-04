using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TrueSync;

public class StackDropTest : TrueSyncBehaviour
{
    [SerializeField]
    private int xSize = 5;
    [SerializeField]
    private int ySize = 5;
    [SerializeField]
    private int zSize = 5;

    [SerializeField]
    private FP xGap = 0.1;
    [SerializeField]
    private FP yGap = 0.1;
    [SerializeField]
    private FP zGap = 0.1;

    private FP boxWidth = 1;

    [SerializeField]
    private GameObject cube;
    [SerializeField]
    private GameObject sphere;
    [SerializeField]
    private GameObject capsule;

    private HashList<GameObject> objStack = new HashList<GameObject>();

    [SerializeField]
    private TSVector offset = TSVector.zero;

    [SerializeField]
    private bool enableLog = false;

    private int frame = 0;

    public override void OnSyncedStart()
    {
        for (int i = 0; i < xSize; i++)
        {
            for (int j = 0; j < ySize; j++)
            {
                for (int k = 0; k < zSize; k++)
                {
                    GameObject tObj = TrueSyncManager.SyncedInstantiate((i+j+k)%2 == 0? sphere : cube, new TSVector(
                        (i - 1) * (boxWidth + xGap) + offset.x,
                        (j - 1) * (boxWidth + yGap) + offset.y,
                        (k - 1) * (boxWidth + zGap) + offset.z), TSQuaternion.AngleAxis(0, TSVector.forward));
                    TSMaterial tMaterial = tObj.AddComponent<TSMaterial>();
                    tMaterial.restitution = 0;
                    tMaterial.friction = 1;
                    objStack.Add(tObj);
                    tObj.SetActive(false);
                }
            }
        }
    }

    public override void OnSyncedUpdate()
    {
        frame++;
        int objSize = objStack.Count;
        FP positionCheckSum = FP.Zero;
        FP rotationCheckSum = FP.Zero;
        for (int i=0; i<objSize; i++)
        {
            GameObject obj = objStack[i];
            obj.SetActive(true);
            TSTransform tTransform = obj.GetComponent<TSTransform>();
            TSVector position = tTransform.position;
            TSQuaternion rotation = tTransform.rotation;
            positionCheckSum += (position.x + position.y + position.z);
            rotationCheckSum += (rotation.x + rotation.y + rotation.z + rotation.w);
        }

        if (enableLog)
            Debug.Log("frame :" + frame + " positionCheckSum: " + positionCheckSum.ToString("F8") + " rotationCheckSum: " + rotationCheckSum.ToString("F8"));
    }
}
