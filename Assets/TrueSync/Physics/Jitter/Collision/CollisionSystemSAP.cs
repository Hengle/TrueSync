/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

#region Using Statements
using System;
using System.Collections.Generic;
#endregion

namespace TrueSync.Physics3D {

    /// <summary>
    /// Uses single axis sweep and prune broadphase collision detection.
    /// </summary>
    public class CollisionSystemSAP : CollisionSystem
    {
        private List<IBroadphaseEntity> bodyList = new List<IBroadphaseEntity>();
        private List<IBroadphaseEntity> active = new List<IBroadphaseEntity>();

        private class IBroadphaseEntityXCompare : IComparer<IBroadphaseEntity>
        {
            public int Compare(IBroadphaseEntity body1, IBroadphaseEntity body2)
            {
                FP f = body1.BoundingBox.min.x - body2.BoundingBox.min.x;
                return (f < 0) ? -1 : (f > 0) ? 1 : 0;
            }
        }

        private IBroadphaseEntityXCompare xComparer;

        /// <summary>
        /// Creates a new instance of the CollisionSystemSAP class.
        /// </summary>
        public CollisionSystemSAP()
        {
            xComparer = new IBroadphaseEntityXCompare();
        }

        /// <summary>
        /// Remove a body from the collision system. Removing a body from the world
        /// does automatically remove it from the collision system.
        /// </summary>
        /// <param name="body">The body to remove.</param>
        /// <returns>Returns true if the body was successfully removed, otherwise false.</returns>
        public override bool RemoveEntity(IBroadphaseEntity body)
        {
            return bodyList.Remove(body);
        }

        /// <summary>
        /// Add a body to the collision system. Adding a body to the world
        /// does automatically add it to the collision system.
        /// </summary>
        /// <param name="body">The body to remove.</param>
        public override void AddEntity(IBroadphaseEntity body)
        {
            if (bodyList.Contains(body))
                throw new ArgumentException("The body was already added to the collision system.", "body");

            bodyList.Add(body);
        }

        /// <summary>
        /// Tells the collisionsystem to check all bodies for collisions. Hook into the
        /// <see cref="CollisionSystem.PassedBroadphase"/>
        /// and <see cref="CollisionSystem.CollisionDetected"/> events to get the results.
        /// </summary>
        public override void Detect()
        {
            bodyList.Sort(xComparer);

            active.Clear();

            for (int i = 0; i < bodyList.Count; i++)
                AddToActive(bodyList[i], false);
        }

        #region private void AddToActiveSingleThreaded(IBroadphaseEntity body, bool addToList)
        private void AddToActive(IBroadphaseEntity body, bool addToList)
        {
            FP xmin = body.BoundingBox.min.x;
            int n = active.Count;

            bool thisInactive = body.IsStaticOrInactive;

            TSBBox acBox, bodyBox;

            for (int i = 0; i != n; )
            {
                IBroadphaseEntity ac = active[i];
                acBox = ac.BoundingBox;

                if (acBox.max.x < xmin)
                {
                    n--;
                    active.RemoveAt(i);
                }
                else
                {
                    bodyBox = body.BoundingBox;

                    if (!(thisInactive && ac.IsStaticOrInactive) &&
                        (((bodyBox.max.z >= acBox.min.z) && (bodyBox.min.z <= acBox.max.z)) &&
                        ((bodyBox.max.y >= acBox.min.y) && (bodyBox.min.y <= acBox.max.y))))
                    {
                        if (base.RaisePassedBroadphase(ac, body))
                        {
                            Detect(body, ac);
                        }
                    }

                    i++;
                }
            }

            active.Add(body);
        }
        #endregion

        private int Compare(IBroadphaseEntity body1, IBroadphaseEntity body2)
        {
            FP f = body1.BoundingBox.min.x - body2.BoundingBox.min.x;
            return (f < 0) ? -1 : (f > 0) ? 1 : 0;
        }

        /// <summary>
        /// Sends a ray (definied by start and direction) through the scene (all bodies added).
        /// NOTE: For performance reasons terrain and trianglemeshshape aren't checked
        /// against rays (rays are of infinite length). They are checked against segments
        /// which start at rayOrigin and end in rayOrigin + rayDirection.
        /// </summary>
        #region public override bool Raycast(JVector rayOrigin, JVector rayDirection, out JVector normal,out FP fraction)
        public override bool Raycast(TSVector rayOrigin, TSVector rayDirection, RaycastCallback raycast, out RigidBody body, out TSVector normal, out FP fraction)
        {
            body = null; normal = TSVector.zero; fraction = FP.MaxValue;

            TSVector tempNormal; FP tempFraction;
            bool result = false;

            // TODO: This can be done better in CollisionSystemPersistenSAP
            foreach (IBroadphaseEntity e in bodyList)
            {
                if (e is SoftBody)
                {
                    SoftBody softBody = e as SoftBody;
                    foreach (RigidBody b in softBody.VertexBodies)
                    {
                        if (this.Raycast(b, rayOrigin, rayDirection, out tempNormal, out tempFraction))
                        {
                            if (tempFraction < fraction && (raycast == null || raycast(b, tempNormal, tempFraction)))
                            {
                                body = b;
                                normal = tempNormal;
                                fraction = tempFraction;
                                result = true;
                            }
                        }
                    }
                }
                else
                {
                    RigidBody b = e as RigidBody;

                    if (this.Raycast(b, rayOrigin, rayDirection, out tempNormal, out tempFraction))
                    {
                        if (tempFraction < fraction && (raycast == null || raycast(b, tempNormal, tempFraction)))
                        {
                            body = b;
                            normal = tempNormal;
                            fraction = tempFraction;
                            result = true;
                        }
                    }
                }
            }

            return result;
        }
        #endregion

        /// <summary>
        /// Sends a ray (definied by start and direction) through the scene (all bodies added).
        /// NOTE: For performance reasons terrain and trianglemeshshape aren't checked
        /// against rays (rays are of infinite length). They are checked against segments
        /// which start at rayOrigin and end in rayOrigin + rayDirection.
        /// </summary>
        public override bool Raycast(TSVector rayOrigin, TSVector rayDirection, RaycastCallback raycast, int layerMask, out RigidBody body, out TSVector normal, out FP fraction)
        {
            body = null; normal = TSVector.zero; fraction = FP.MaxValue;

            TSVector tempNormal; FP tempFraction;
            bool result = false;

            // TODO: This can be done better in CollisionSystemPersistenSAP
            foreach (IBroadphaseEntity e in bodyList)
            {
                if (e is SoftBody)
                {
                    SoftBody softBody = e as SoftBody;
                    foreach (RigidBody b in softBody.VertexBodies)
                    {
                        int bodyLayerMask = 1 << PhysicsManager.instance.GetBodyLayer(b);
                        if ((layerMask & bodyLayerMask) != bodyLayerMask)
                            continue;

                        if (this.Raycast(b, rayOrigin, rayDirection, out tempNormal, out tempFraction))
                        {
                            if (tempFraction < fraction && (raycast == null || raycast(b, tempNormal, tempFraction)))
                            {
                                body = b;
                                normal = tempNormal;
                                fraction = tempFraction;
                                result = true;
                            }
                        }
                    }
                }
                else
                {
                    RigidBody b = e as RigidBody;
                    int bodyLayerMask = 1 << PhysicsManager.instance.GetBodyLayer(b);
                    if ((layerMask & bodyLayerMask) != bodyLayerMask)
                        continue;

                    if (this.Raycast(b, rayOrigin, rayDirection, out tempNormal, out tempFraction))
                    {
                        if (tempFraction < fraction && (raycast == null || raycast(b, tempNormal, tempFraction)))
                        {
                            body = b;
                            normal = tempNormal;
                            fraction = tempFraction;
                            result = true;
                        }
                    }
                }
            }

            return result;
        }
    }
}
