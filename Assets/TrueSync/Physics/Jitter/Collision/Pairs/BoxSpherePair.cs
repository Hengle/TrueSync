using System.Collections;
using System.Collections.Generic;

namespace TrueSync.Physics3D
{
    public class BoxSpherePair : CollisionPair
    {
        public static ResourcePool<BoxSpherePair> pool = new ResourcePool<BoxSpherePair>();

        private static FP[] boxMin = new FP[3];
        private static FP[] boxMax = new FP[3];
        private static FP[] point = new FP[3];
        private static FP[] closestPoint = new FP[3];

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
                point = point1;
                penetration = -dist;
                return true;
            }
            return false;
        }

        public FP GetSphereDistance(ref BoxShape box, ref TSVector boxPosition, ref TSMatrix boxOrientation, ref TSVector sphereCenter, FP radius,
            ref TSVector pointOnBox, ref TSVector pointOnSphere, ref TSVector normal)
        {
            TSVector boxHalfSize = box.halfSize;
            boxMin[0] = -boxHalfSize.x;
            boxMin[1] = -boxHalfSize.y;
            boxMin[2] = -boxHalfSize.z;

            boxMax[0] = boxHalfSize.x;
            boxMax[1] = boxHalfSize.y;
            boxMax[2] = boxHalfSize.z;

            TSVector prel;
            // convert sphere center into box local space
            TSVector.Subtract(ref sphereCenter, ref boxPosition, out prel);
            TSMatrix invBoxOrientation;
            TSMatrix.Inverse(ref boxOrientation, out invBoxOrientation);
            TSVector.Transform(ref prel, ref invBoxOrientation, out prel);

            point[0] = prel.x;
            point[1] = prel.y;
            point[2] = prel.z;

            for (int i = 0; i < 3; i++)
            {
                FP v = point[i];
                v = TSMath.Max(v, boxMin[i]);
                v = TSMath.Min(v, boxMax[i]);
                closestPoint[i] = v;
            }

            pointOnBox.x = closestPoint[0];
            pointOnBox.y = closestPoint[1];
            pointOnBox.z = closestPoint[2];

            normal = prel - pointOnBox;
            FP sqrDist = normal.sqrMagnitude;
            if (sqrDist < radius * radius)
            {
                // transform back in world space
                TSVector.Transform(ref pointOnBox, ref boxOrientation, out pointOnBox);
                TSVector.Add(ref pointOnBox, ref boxPosition, out pointOnBox);
                normal = pointOnBox - sphereCenter;
                normal.Normalize();
                pointOnSphere = sphereCenter + normal * radius;
                return TSMath.Sqrt(sqrDist) - radius;
            }
            return FP.One;
        }
    }
}
