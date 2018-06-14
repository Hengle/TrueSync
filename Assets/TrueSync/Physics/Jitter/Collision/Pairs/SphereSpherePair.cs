using System.Collections;
using System.Collections.Generic;

namespace TrueSync.Physics3D
{
    public class SphereSpherePair : CollisionPair
    {
        public static ResourcePool<SphereSpherePair> pool = new ResourcePool<SphereSpherePair>();

        public override bool IsColliding(ref TSMatrix orientation1, ref TSMatrix orientation2, ref TSVector position1, ref TSVector position2, 
            out TSVector point, out TSVector point1, out TSVector point2, out TSVector normal, out FP penetration)
        {
            // Used variables
            TSVector center1, center2;

            // Initialization of the output
            point = point1 = point2 = normal = TSVector.zero;
            penetration = FP.Zero;

            SphereShape sphere1 = this.Shape1 as SphereShape;
            SphereShape sphere2 = this.Shape2 as SphereShape;

            // Get the center of sphere1 in world coordinates -> center1
            sphere1.SupportCenter(out center1);
            TSVector.Transform(ref center1, ref orientation1, out center1);
            TSVector.Add(ref position1, ref center1, out center1);

            // Get the center of sphere2 in world coordinates -> center2
            sphere2.SupportCenter(out center2);
            TSVector.Transform(ref center2, ref orientation2, out center2);
            TSVector.Add(ref position2, ref center2, out center2);

            TSVector c12 = TSVector.Subtract(center1, center2);
            FP dot = TSVector.Dot(c12, c12);
            FP r = sphere1.radius + sphere2.radius;
            if (dot <= r * r)
            {
                //Get the unit direction from the first sphere's center to the second sphere's center.
                TSVector.Subtract(ref center2, ref center1, out normal);
                if (normal.sqrMagnitude < TSMath.Epsilon)
                {
                    // Spheres are on the same position, we can choose any normal vector.
                    // Possibly it would be better to consider the object movement (velocities), but 
                    // it is not important since this case should be VERY rare.
                    normal = TSVector.forward;
                }
                else
                    normal = normal.normalized;

                FP r1 = sphere1.radius;
                FP r2 = sphere2.radius;

                point1 = normal * r1 + center1;
                point2 = TSVector.Negate(normal) * r2 + center2;

                TSVector.Negate(ref normal, out normal);
                penetration = r - TSMath.Sqrt(dot);
                return true;
            }
            return false;
        }
    }
}
