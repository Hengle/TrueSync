using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TrueSync;
using System;

public class MoveCube : TrueSyncBehaviour
{
    private const byte FORWARD = 2;
    private const byte BACKWARD = 4;
    private const byte RIGHT = 8;
    private const byte LEFT = 16;

    [SerializeField]
    private FP forceFactor = FP.One;

    private bool isCollide = false;

    public override void OnSyncedInput()
    {
        TrueSyncInput.SetBool(FORWARD, Input.GetKey(KeyCode.UpArrow));
        TrueSyncInput.SetBool(BACKWARD, Input.GetKey(KeyCode.DownArrow));
        TrueSyncInput.SetBool(RIGHT, Input.GetKey(KeyCode.RightArrow));
        TrueSyncInput.SetBool(LEFT, Input.GetKey(KeyCode.LeftArrow));
    }

    public override void OnSyncedUpdate()
    {
        bool goForward = TrueSyncInput.GetBool(FORWARD);
        bool goBackward = TrueSyncInput.GetBool(BACKWARD);
        bool goRight = TrueSyncInput.GetBool(RIGHT);
        bool goLeft = TrueSyncInput.GetBool(LEFT);

        if (goForward && !isCollide)
            tsRigidBody.AddForce(TSVector.forward * forceFactor);

        if (goBackward && !isCollide)
            tsRigidBody.AddForce(TSVector.Negate(TSVector.forward) * forceFactor);

        if (goRight && !isCollide)
            tsRigidBody.AddForce(TSVector.right * forceFactor);

        if (goLeft && !isCollide)
            tsRigidBody.AddForce(TSVector.Negate(TSVector.right) * forceFactor);

        isCollide = false;
    }

    public override void OnSyncedCollisionEnter(TSCollision other)
    {
        isCollide = other.collider is TSBoxCollider;
    }

    //public override void OnSyncedCollisionExit(TSCollision other)
    //{
    //    isCollide = false;
    //}
}
