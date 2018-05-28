using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TrueSync;
using FixPointCS;
using UnityEngine.Profiling;

public class FixedPointTest : MonoBehaviour
{
    private int iteration = 10000;
    private FP testFP = 5;
    private F64 testF64 = new F64(5);

    private TSVector2 testTSVector = TSVector2.one;
    private F64Vec2 testFPVec2 = F64Vec2.One;
    // Use this for initialization
    void Start()
    {
        Debug.Log("TrueSync: " + (testFP / testFP).ToString());
        Debug.Log("FixPointCs: " + (testF64 / testF64).ToString());
    }

    // Update is called once per frame
    void Update()
    {
        //TrueSync FixedPoint
        Profiler.BeginSample("TrueSync FixedPoint");
        for (int i = 0; i < iteration; i++)
        {
            //FP.Sqrt(testFP);
            //FP.Cos(testFP);
            //FP test = testFP * testFP;
            //TSVector2.Dot(testTSVector, testTSVector);
            //FP.FastSin(testFP);
            //FP.FastMul(testFP, testFP);
            //FP test = testFP / testFP;
            TSMath.Pow(testFP, testFP);
        }
        Profiler.EndSample();

        //FixPointCs
        Profiler.BeginSample("FixPointCs");
        for (int i = 0; i < iteration; i++)
        {
            //F64.SqrtFastest(testF64);
            //F64.Cos(testF64);
            //F64 test = Fixed64.Multi(testF64.raw, testF64.raw); //testF64 * testF64;
            //F64Vec2.Dot(testF64Vec2, testF64Vec2);
            //F64.SinFastest(testF64);
            //F64.FromRaw(Fixed64.Mul(testF64.raw, testF64.raw));
            //F64.FromRaw(Fixed64.DivFastest(testF64.raw, testF64.raw));
            //F64 test = testF64 / testF64;
            F64.Pow(testF64, testF64);
        }
        Profiler.EndSample();
    }
}
