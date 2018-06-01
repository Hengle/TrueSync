using System.Collections;
using System.Collections.Generic;

namespace TrueSync.Physics3D
{
    public class BoxSpherePair : CollisionPair
    {
        public static ResourcePool<BoxSpherePair> pool = new ResourcePool<BoxSpherePair>();

        public override bool IsColliding(ref TSMatrix orientation1, ref TSMatrix orientation2, ref TSVector position1, ref TSVector position2,
            out TSVector point, out TSVector point1, out TSVector point2, out TSVector normal, out FP penetration)
        {
            // Used variables
            TSVector center1, center2;

            // Initialization of the output
            point = point1 = point2 = normal = TSVector.zero;
            penetration = FP.Zero;

            BoxShape box = this.Shape1 as BoxShape;
            SphereShape sphere = this.Shape2 as SphereShape;

            // Get the center of box in world coordinates -> center1
            box.SupportCenter(out center1);
            TSVector.Transform(ref center1, ref orientation1, out center1);
            TSVector.Add(ref position1, ref center1, out center1);

            // Get the center of sphere in world coordinates -> center2
            sphere.SupportCenter(out center2);
            TSVector.Transform(ref center2, ref orientation2, out center2);
            TSVector.Add(ref position2, ref center2, out center2);

            FP dist = GetSphereDistance(ref box, ref center1, ref orientation1, ref center2, sphere.radius,
                ref point1, ref point2, ref normal);

            if (dist < TSMath.Epsilon)
            {
                penetration = -dist;
                point = point1;
                return true;
            }
            return false;
        }

        public FP GetSphereDistance(ref BoxShape box, ref TSVector boxPosition, ref TSMatrix boxOrientation, ref TSVector sphereCenter, FP radius,
            ref TSVector pointOnBox, ref TSVector pointOnSphere, ref TSVector normal)
        {
            TSVector boxHalfSize = box.halfSize;
            TSVector prel;
            // convert sphere center into box local space
            TSVector.Subtract(ref sphereCenter, ref boxPosition, out prel);
            TSMatrix invBoxOrientation;
            TSMatrix.Inverse(ref boxOrientation, out invBoxOrientation);
            TSVector.Transform(ref prel, ref invBoxOrientation, out prel);

            pointOnBox = prel;
            bool outSide = false;
            //x
            if (prel.x < -boxHalfSize.x)
            {
                outSide = true;
                pointOnBox.x = -boxHalfSize.x;
            }
            else if (prel.x > boxHalfSize.x)
            {
                outSide = true;
                pointOnBox.x = boxHalfSize.x;
            }

            //y
            if (prel.y < -boxHalfSize.y)
            {
                outSide = true;
                pointOnBox.y = -boxHalfSize.y;
            }
            else if (prel.y > boxHalfSize.y)
            {
                outSide = true;
                pointOnBox.y = boxHalfSize.y;
            }

            //z
            if (prel.z < -boxHalfSize.z)
            {
                outSide = true;
                pointOnBox.z = -boxHalfSize.z;
            }
            else if (prel.z > boxHalfSize.z)
            {
                outSide = true;
                pointOnBox.z = boxHalfSize.z;
            }

            if (outSide)
            {
                normal = prel - pointOnBox;

                FP dist2 = normal.sqrMagnitude;
                if (dist2 > radius * radius)
                {
                    return FP.One;
                }

                FP distance;
                if (dist2 <= TSMath.Epsilon)
                {
                    distance = -GetSpherePenetration(boxHalfSize, prel, ref pointOnBox, out normal);
                }
                else
                {
                    distance = normal.magnitude;
                    normal /= distance;
                }
                // transform back in world space
                TSVector.Transform(ref pointOnBox, ref boxOrientation, out pointOnBox);
                TSVector.Add(ref pointOnBox, ref boxPosition, out pointOnBox);

                pointOnSphere = prel - normal * radius;
                TSVector.Transform(ref pointOnSphere, ref boxOrientation, out pointOnSphere);
                TSVector.Add(ref pointOnSphere, ref boxPosition, out pointOnSphere);

                normal = TSVector.Transform(normal, boxOrientation);
                normal = TSVector.Negate(normal);
                normal.Normalize();

                return distance - radius;
            }
            else
            {
                FP distance;
                distance = -GetSpherePenetration(boxHalfSize, prel, ref pointOnBox, out normal);
                normal = TSVector.Transform(normal, boxOrientation);
                normal = TSVector.Negate(normal);
                pointOnSphere = sphereCenter + normal * radius;
                TSVector.Transform(ref pointOnBox, ref boxOrientation, out pointOnBox);
                TSVector.Add(ref pointOnBox, ref boxPosition, out pointOnBox);
                return distance - radius;
            }
        }

        private FP GetSpherePenetration(TSVector boxHalfExtent, TSVector sphereRelPos, ref TSVector closestPoint, out TSVector normal)
        {
            //project the center of the sphere on the closest face of the box
            FP faceDist = boxHalfExtent.x - sphereRelPos.x;
            FP minDist = faceDist;
            closestPoint.x = boxHalfExtent.x;
            normal = TSVector.right;

            faceDist = boxHalfExtent.x + sphereRelPos.x;
            if (faceDist < minDist)
            {
                minDist = faceDist;
                closestPoint = sphereRelPos;
                closestPoint.x = -boxHalfExtent.x;
                normal = TSVector.Negate(TSVector.right);
            }

            faceDist = boxHalfExtent.y - sphereRelPos.y;
            if (faceDist < minDist)
            {
                minDist = faceDist;
                closestPoint = sphereRelPos;
                closestPoint.y = boxHalfExtent.y;
                normal = TSVector.up;
            }

            faceDist = boxHalfExtent.y + sphereRelPos.y;
            if (faceDist < minDist)
            {
                minDist = faceDist;
                closestPoint = sphereRelPos;
                closestPoint.y = -boxHalfExtent.y;
                normal = TSVector.Negate(TSVector.up);
            }

            faceDist = boxHalfExtent.z - sphereRelPos.z;
            if (faceDist < minDist)
            {
                minDist = faceDist;
                closestPoint = sphereRelPos;
                closestPoint.z = boxHalfExtent.z;
                normal = TSVector.forward;
            }

            faceDist = boxHalfExtent.z + sphereRelPos.z;
            if (faceDist < minDist)
            {
                minDist = faceDist;
                closestPoint = sphereRelPos;
                closestPoint.z = -boxHalfExtent.z;
                normal = TSVector.Negate(TSVector.forward);
            }

            return minDist;
        }
    }
}
