using System.Collections;
using System.Collections.Generic;

namespace TrueSync.Physics3D
{
    public class CapsuleSpherePair : CollisionPair
    {
        public static ResourcePool<CapsuleSpherePair> pool = new ResourcePool<CapsuleSpherePair>();

        public override bool IsColliding(ref TSMatrix orientation1, ref TSMatrix orientation2, ref TSVector position1, ref TSVector position2,
            out TSVector point, out TSVector point1, out TSVector point2, out TSVector normal, out FP penetration)
        {
            // Used variables
            TSVector center1, center2;

            // Initialization of the output
            point = point1 = point2 = normal = TSVector.zero;
            penetration = FP.Zero;

            CapsuleShape capsule = this.Shape1 as CapsuleShape;
            SphereShape sphere = this.Shape2 as SphereShape;

            // Get the center of capsule in world coordinates -> center1
            capsule.SupportCenter(out center1);
            TSVector.Transform(ref center1, ref orientation1, out center1);
            TSVector.Add(ref position1, ref center1, out center1);

            // Get the center of sphere in world coordinates -> center2
            sphere.SupportCenter(out center2);
            TSVector.Transform(ref center2, ref orientation2, out center2);
            TSVector.Add(ref position2, ref center2, out center2);

            TSVector aP1, aP2, axisA;
            TSVector halfAxisVecA = TSVector.up * FP.Half * capsule.length;
            TSVector.Transform(ref halfAxisVecA, ref orientation1, out axisA);
            TSVector.Add(ref center1, ref axisA, out aP1);
            TSVector.Negate(ref axisA, out axisA);
            TSVector.Add(ref center1, ref axisA, out aP2);

            return OverlapTest(ref capsule, ref aP1, ref aP2, ref sphere, ref center2,
                ref point1, ref point2, ref normal, ref penetration);
        }

        private bool OverlapTest(ref CapsuleShape capsule, ref TSVector p1, ref TSVector p2, 
            ref SphereShape sphere, ref TSVector sphereCenter,
            ref TSVector pa, ref TSVector pb, ref TSVector normal, ref FP penetration)
        {
            SegmentShape cap = SegmentShape.Pool.GetNew();
            cap.P1 = p1;
            cap.P2 = p2;
            TSVector v;
            FP r2 = capsule.Radius + sphere.Radius;
            r2 *= r2;

            FP sb;
            cap.ClosestPointTo(ref sphereCenter, out sb, out pb);
            SegmentShape.Pool.GiveBack(cap);

            TSVector.Subtract(ref sphereCenter, ref pb, out normal);
            if (normal.sqrMagnitude - r2 >= TSMath.Epsilon)
                return false;

            penetration = (capsule.radius + sphere.radius) - normal.magnitude;
            normal.Normalize();
            TSVector.Multiply(ref normal, -sphere.Radius, out v);
            TSVector.Add(ref sphereCenter, ref v, out pa);
            TSVector.Multiply(ref normal, capsule.Radius, out v);
            TSVector.Add(ref pb, ref v, out pb);

            TSVector.Negate(ref normal, out normal);
            return true;
        }

        //public override void SweptTest(CollisionFunctor cf, Part partA, Part partB, TSVector delta)
        //{
        //    var a = (SpherePart)partA;
        //    var b = (CapsulePart)partB;

        //    SegmentShape path;
        //    path.P1 = a.World.Center;
        //    TSVector.Add(ref path.P1, ref delta, out path.P2);

        //    CapsuleShape cap = b.World;
        //    cap.Radius += a.World.Radius;
        //    SegmentShape capSegment = new SegmentShape(b.World.P1, b.World.P2);

        //    FP k;
        //    TSVector pa, pb, normal;
        //    cap.Intersect(ref path, out k, out pa);
        //    if (k <= FP.One)
        //    {
        //        capSegment.ClosestPointTo(ref pa, out k, out pb);
        //        TSVector.Subtract(ref pa, ref pb, out normal);
        //        normal.Normalize();
        //        TSVector.Multiply(ref normal, b.World.Radius, out pa);
        //        TSVector.Add(ref pb, ref pa, out pb);
        //        TSVector.Multiply(ref normal, -a.World.Radius, out pa);
        //        TSVector.Add(ref a.World.Center, ref pa, out pa);

        //        cf.WritePoint(ref pa, ref pb, ref normal);
        //    }
        //}
    }
}
