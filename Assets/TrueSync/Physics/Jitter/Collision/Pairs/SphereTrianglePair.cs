using System.Collections;
using System.Collections.Generic;

namespace TrueSync.Physics3D
{
    public class SphereTrianglePair : CollisionPair
    {
        public static ResourcePool<SphereTrianglePair> pool = new ResourcePool<SphereTrianglePair>();

        public override bool IsColliding(ref TSMatrix orientation1, ref TSMatrix orientation2, ref TSVector position1, ref TSVector position2,
            out TSVector point, out TSVector point1, out TSVector point2, out TSVector normal, out FP penetration)
        {
            // Used variables
            TSVector center1, center2;

            // Initialization of the output
            point = point1 = point2 = normal = TSVector.zero;
            penetration = FP.Zero;

            TriangleMeshShape triangle = this.Shape1 as TriangleMeshShape;
            SphereShape sphere = this.Shape2 as SphereShape;

            // Get the center of sphere in world coordinates -> center1
            triangle.SupportCenter(out center1);
            TSVector.Transform(ref center1, ref orientation1, out center1);
            TSVector.Add(ref position1, ref center1, out center1);

            // Get the center of triangle in world coordinates -> center2
            sphere.SupportCenter(out center2);
            TSVector.Transform(ref center2, ref orientation2, out center2);
            TSVector.Add(ref position2, ref center2, out center2);

            TSVector[] vertices = triangle.Vertices;
            TSVector.Transform(ref vertices[0], ref orientation1, out vertices[0]);
            TSVector.Add(ref position1, ref vertices[0], out vertices[0]);
            TSVector.Transform(ref vertices[1], ref orientation1, out vertices[1]);
            TSVector.Add(ref position1, ref vertices[1], out vertices[1]);
            TSVector.Transform(ref vertices[2], ref orientation1, out vertices[2]);
            TSVector.Add(ref position1, ref vertices[2], out vertices[2]);

            return Collide(center2, sphere.radius, ref vertices, ref point, ref point1, ref point2, ref normal, ref penetration);
        }

        /// <summary>
        /// See also geometrictools.com
        /// Basic idea: D = |p - (lo + t0*lv)| where t0 = lv . (p - lo) / lv . lv
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <param name="p"></param>
        /// <param name="nearest"></param>
        /// <returns></returns>
        private FP SegmentSquareDistance(TSVector from, TSVector to, TSVector point, ref TSVector nearest)
        {
            TSVector diff = point - from;
            TSVector v = to - from;
            FP t = TSVector.Dot(v, diff);

            if (t > FP.Zero)
            {
                FP dotVV = TSVector.Dot(v, v);
                if (t < dotVV)
                {
                    t /= dotVV;
                    diff -= t * v;
                }
                else
                {
                    t = 1;
                    diff -= v;
                }
            }
            else
                t = 0;

            nearest = from + t * v;
            return TSVector.Dot(diff, diff);
        }

        private bool Collide(TSVector sphereCenter, FP r, ref TSVector[] vertices,
            ref TSVector point, ref TSVector point1, ref TSVector point2, ref TSVector resultNormal, ref FP penetration)
        {
            TSVector c = sphereCenter;
            TSVector delta = TSVector.one;

            TSVector normal = TSVector.Cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
            normal = TSVector.Normalize(normal);
            TSVector p1ToCentre = c - vertices[0];
            FP distanceFromPlane = TSVector.Dot(p1ToCentre, normal);

            if (distanceFromPlane < FP.Zero)
            {
                //triangle facing the other way
                distanceFromPlane *= -1;
                normal *= -1;
            }

            FP contactMargin = FP.Zero; //TODO: PersistentManifold.ContactBreakingThreshold;
            bool isInsideContactPlane = distanceFromPlane < r + contactMargin;
            bool isInsideShellPlane = distanceFromPlane < r;

            FP deltaDotNormal = TSVector.Dot(delta, normal);
            if (!isInsideShellPlane && deltaDotNormal >= FP.Zero)
                return false;

            // Check for contact / intersection
            bool hasContact = false;
            TSVector contactPoint = TSVector.zero;
            if (isInsideContactPlane)
            {
                if (FaceContains(ref c, ref vertices, ref normal))
                {
                    // Inside the contact wedge - touches a point on the shell plane
                    hasContact = true;
                    contactPoint = c - normal * distanceFromPlane;
                }
                else
                {
                    // Could be inside one of the contact capsules
                    FP contactCapsuleRadiusSqr = (r + contactMargin) * (r + contactMargin);
                    TSVector nearestOnEdge = TSVector.zero;
                    for (int i = 0; i < 3; i++)
                    {
                        TSVector pa, pb;
                        pa = vertices[i];
                        pb = vertices[(i + 1) % 3];

                        FP distanceSqr = SegmentSquareDistance(pa, pb, c, ref nearestOnEdge);
                        if (distanceSqr < contactCapsuleRadiusSqr)
                        {
                            // Yep, we're inside a capsule
                            contactCapsuleRadiusSqr = distanceSqr;
                            hasContact = true;
                            contactPoint = nearestOnEdge;
                        }
                    }
                }
            }

            if (hasContact)
            {
                FP MaxOverlap = FP.Zero; // TODO:
                TSVector contactToCentre = c - contactPoint;
                FP distanceSqr = contactToCentre.sqrMagnitude;
                if (distanceSqr < (r - MaxOverlap) * (r - MaxOverlap))
                {
                    FP distance = TSMath.Sqrt(distanceSqr);
                    resultNormal = contactToCentre;
                    resultNormal = TSVector.Normalize(resultNormal);
                    point = contactPoint;
                    point1 = point;
                    resultNormal = TSVector.Negate(resultNormal);
                    point2 = sphereCenter + resultNormal * r;
                    penetration = r - distance;
                    return true;
                }

                if (TSVector.Dot(delta, contactToCentre) >= FP.Zero)
                    return false;

                // Moving towards the contact point -> collision
                point = point1 = point2 = contactPoint;
                return true;
            }
            return false;
        }

        private bool PointInTriangle(ref TSVector[] vertices, ref TSVector normal, ref TSVector p)
        {
            TSVector p1 = vertices[0];
            TSVector p2 = vertices[1];
            TSVector p3 = vertices[2];

            TSVector edge1 = p2 - p1;
            TSVector edge2 = p3 - p2;
            TSVector edge3 = p1 - p3;

            TSVector p1ToP = p - p1;
            TSVector p2ToP = p - p2;
            TSVector p3ToP = p - p3;

            TSVector edge1Normal = TSVector.Cross(edge1, normal);
            TSVector edge2Normal = TSVector.Cross(edge2, normal);
            TSVector edge3Normal = TSVector.Cross(edge3, normal);

            FP r1, r2, r3;
            r1 = TSVector.Dot(edge1Normal, p1ToP);
            r2 = TSVector.Dot(edge2Normal, p2ToP);
            r3 = TSVector.Dot(edge3Normal, p3ToP);
            if ((r1 > FP.Zero && r2 > FP.Zero && r3 > FP.Zero) ||
                 (r1 <= FP.Zero && r2 <= FP.Zero && r3 <= FP.Zero))
                return true;
            return false;
        }

        private bool FaceContains(ref TSVector p, ref TSVector[] vertices, ref TSVector normal)
        {
            TSVector lp = p;
            TSVector lnormal = normal;
            return PointInTriangle(ref vertices, ref lnormal, ref lp);
        }
    }
}
