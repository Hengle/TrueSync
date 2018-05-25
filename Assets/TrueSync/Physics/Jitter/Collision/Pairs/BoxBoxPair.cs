using System.Collections;
using System.Collections.Generic;

namespace TrueSync.Physics3D
{
    public class BoxBoxPair : CollisionPair
    {
        public static ResourcePool<BoxBoxPair> pool = new ResourcePool<BoxBoxPair>();

        TSVector[] aAxes;
        TSVector[] bAxes;
        TSVector[] AllAxes;
        TSVector[] aVertices;
        TSVector[] bVertices;

        FP minOverlap = FP.Zero;
        TSVector minOverlapAxis = TSVector.zero;

        //List<TSVector> penetrationAxes = new List<TSVector>();
        //List<FP> penetrationAxesDistance = new List<FP>();

        TSVector penetrationAxis = TSVector.zero;
        FP penetrationAxisDistance = FP.Zero;

        public override bool IsColliding(ref TSMatrix orientation1, ref TSMatrix orientation2, ref TSVector position1, ref TSVector position2,
            out TSVector point, out TSVector point1, out TSVector point2, out TSVector normal, out FP penetration)
        {
            // Used variables
            TSVector center1, center2;

            // Initialization of the output
            point = point1 = point2 = normal = TSVector.zero;
            penetration = FP.Zero;

            BoxShape box1 = this.Shape1 as BoxShape;
            BoxShape box2 = this.Shape2 as BoxShape;

            // Get the center of box1 in world coordinates -> center1
            box1.SupportCenter(out center1);
            TSVector.Transform(ref center1, ref orientation1, out center1);
            TSVector.Add(ref position1, ref center1, out center1);

            // Get the center of box2 in world coordinates -> center2
            box2.SupportCenter(out center2);
            TSVector.Transform(ref center2, ref orientation2, out center2);
            TSVector.Add(ref position2, ref center2, out center2);

            TSVector[] aAxes = GetAxes(ref box1, ref orientation1);
            TSVector[] bAxes = GetAxes(ref box2, ref orientation2);

            AllAxes = new TSVector[]
            {
                aAxes[0],
                aAxes[1],
                aAxes[2],
                bAxes[0],
                bAxes[1],
                bAxes[2],
                TSVector.Cross(aAxes[0], bAxes[0]),
                TSVector.Cross(aAxes[0], bAxes[1]),
                TSVector.Cross(aAxes[0], bAxes[2]),
                TSVector.Cross(aAxes[1], bAxes[0]),
                TSVector.Cross(aAxes[1], bAxes[1]),
                TSVector.Cross(aAxes[1], bAxes[2]),
                TSVector.Cross(aAxes[2], bAxes[0]),
                TSVector.Cross(aAxes[2], bAxes[1]),
                TSVector.Cross(aAxes[2], bAxes[2])
            };

            aVertices = GetVertices(ref box1, ref center1, ref orientation1);
            bVertices = GetVertices(ref box2, ref center2, ref orientation2);
            int aVertsLength = aVertices.Length;
            int bVertsLength = bVertices.Length;

            //penetrationAxes.Clear();
            //penetrationAxesDistance.Clear();

            bool hasOverlap = false;

            if (ProjectionHasOverlap(AllAxes.Length, AllAxes, bVertsLength, bVertices, aVertsLength, aVertices))
            {
                hasOverlap = true;
            }
            else if (ProjectionHasOverlap(AllAxes.Length, AllAxes, aVertsLength, aVertices, bVertsLength, bVertices))
            {
                hasOverlap = true;
            }

            if (hasOverlap)
            {
                normal = penetrationAxis.normalized;
                penetration = penetrationAxisDistance;

                point.x = normal.x * box1.halfSize.x + center1.x;
                point.y = normal.y * box1.halfSize.y + center1.y;
                point.z = normal.z * box1.halfSize.z + center1.z;

                //point2.x = -normal.x * box2.halfSize.x + center2.x;
                //point2.y = -normal.y * box2.halfSize.y + center2.y;
                //point2.z = -normal.z * box2.halfSize.z + center2.z;
            }

            return hasOverlap;
        }

        /// Detects whether or not there is overlap on all separating axes.
        private bool ProjectionHasOverlap(int aAxesLength, TSVector[] aAxes,
            int bVertsLength, TSVector[] bVertices, int aVertsLength, TSVector[] aVertices)
        {
            minOverlap = FP.PositiveInfinity;

            for (int i = 0; i < aAxesLength; i++)
            {


                FP bProjMin = FP.MaxValue, aProjMin = FP.MaxValue;
                FP bProjMax = FP.MinValue, aProjMax = FP.MinValue;

                TSVector axis = aAxes[i];

                // Handles the cross product = {0,0,0} case
                if (aAxes[i] == TSVector.zero)
                    return true;

                for (int j = 0; j < bVertsLength; j++)
                {
                    FP val = FindScalarProjection((bVertices[j]), axis);

                    if (val < bProjMin)
                    {
                        bProjMin = val;
                    }

                    if (val > bProjMax)
                    {
                        bProjMax = val;
                    }
                }

                for (int j = 0; j < aVertsLength; j++)
                {
                    FP val = FindScalarProjection((aVertices[j]), axis);

                    if (val < aProjMin)
                    {
                        aProjMin = val;
                    }

                    if (val > aProjMax)
                    {
                        aProjMax = val;
                    }
                }

                FP overlap = FindOverlap(aProjMin, aProjMax, bProjMin, bProjMax);

                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    minOverlapAxis = axis;

                    //penetrationAxes.Add(axis);
                    //penetrationAxesDistance.Add(overlap);
                    penetrationAxis = axis;
                    penetrationAxisDistance = overlap;

                }

                if (overlap <= 0)
                {
                    // Separating Axis Found Early Out
                    return false;
                }
            }

            return true; // A penetration has been found
        }


        /// Calculates the scalar projection of one vector onto another, assumes normalised axes
        private static FP FindScalarProjection(TSVector point, TSVector axis)
        {
            return TSVector.Dot(point, axis);
        }

        /// Calculates the amount of overlap of two intervals.
        private FP FindOverlap(FP astart, FP aend, FP bstart, FP bend)
        {
            if (astart < bstart)
            {
                if (aend < bstart)
                {
                    return FP.Zero;
                }

                return aend - bstart;
            }

            if (bend < astart)
            {
                return FP.Zero;
            }

            return bend - astart;
        }

        private TSVector[] GetAxes(ref BoxShape box, ref TSMatrix orientation)
        {
            TSVector[] axes = new[]
            {
                TSVector.Transform(TSVector.right, orientation),
                TSVector.Transform(TSVector.up, orientation),
                TSVector.Transform(TSVector.forward, orientation)
            };
            return axes;
        }

        private TSVector[] GetVertices(ref BoxShape box, ref TSVector center, ref TSMatrix orientation)
        {
            TSVector[] cubeVertices =
            {
                new TSVector(-FP.One, -FP.One, -FP.One),
                new TSVector(-FP.One, -FP.One, FP.One),
                new TSVector(-FP.One, FP.One, -FP.One),
                new TSVector(-FP.One, FP.One, FP.One),
                new TSVector(FP.One, -FP.One, -FP.One),
                new TSVector(FP.One, -FP.One, FP.One),
                new TSVector(FP.One, FP.One, -FP.One),
                new TSVector(FP.One, FP.One, FP.One)
            };

            TSVector halfSize = box.halfSize;
            for (int i = 0; i < 8; i++)
            {
                TSVector vertex = cubeVertices[i];
                vertex.x *= halfSize.x;
                vertex.y *= halfSize.y;
                vertex.z *= halfSize.z;
                TSVector.Transform(ref vertex, ref orientation, out cubeVertices[i]);
                TSVector.Add(ref cubeVertices[i], ref center, out cubeVertices[i]);
            }

            return cubeVertices;
        }
    }
}
