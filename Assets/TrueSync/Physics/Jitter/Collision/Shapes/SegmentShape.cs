using System.Collections;
using System.Collections.Generic;

namespace TrueSync.Physics3D
{
    /// <summary>
    /// Represents a line segment as defined by two distinct endpoints.
    /// </summary>
    public class SegmentShape : Shape, ResourcePoolItem
    {
        public static ResourcePool<SegmentShape> Pool = new ResourcePool<SegmentShape>();

        public TSVector P1;
        public TSVector P2;

        /// <summary>
        /// Construct a new line segment.
        /// </summary>
        /// <param name="p1">The first endpoint.</param>
        /// <param name="p2">The second endpoint.</param>
        public SegmentShape(TSVector p1, TSVector p2)
        {
            P1 = p1;
            P2 = p2;
        }

        public SegmentShape()
        {
            P1 = TSVector.zero;
            P2 = TSVector.zero;
        }

        /// <summary>
        /// Gets the squared distance between the given point and the segment.
        /// </summary>
        /// <param name="p">The point with which to calculate the distance.</param>
        /// <returns>Returns the squared distance.</returns>
        public FP DistanceSquaredTo(ref TSVector p)
        {
            TSVector pq, pp, qp;
            FP e, f;

            TSVector.Subtract(ref P2, ref P1, out pq);
            TSVector.Subtract(ref p, ref P1, out pp);
            TSVector.Subtract(ref p, ref P2, out qp);

            e = TSVector.Dot(ref pp, ref pq);
            if (e <= FP.Zero)
                return pp.sqrMagnitude;

            f = TSVector.Dot(ref pq, ref pq);
            if (e >= f)
                return qp.sqrMagnitude;

            return pp.sqrMagnitude - e * e / f;
        }

        /// <summary>
        /// Gets the closest point on the segment to the given point.
        /// </summary>
        /// <param name="p">The point with which to calculate the nearest segment point.</param>
        /// <param name="scalar">Returns a value between 0 and 1 indicating the location on the segment nearest the point.</param>
        /// <param name="output">Returns the closest point on the segment.</param>
        public void ClosestPointTo(ref TSVector p, out FP scalar, out TSVector output)
        {
            TSVector u, v;
            TSVector.Subtract(ref p, ref P1, out v);
            TSVector.Subtract(ref P2, ref P1, out u);
            scalar = TSVector.Dot(ref u, ref v);
            scalar /= u.sqrMagnitude;
            if (scalar <= FP.Zero)
                output = P1;
            else if (scalar >= FP.One)
                output = P2;
            else
            {
                TSVector.Multiply(ref u, scalar, out output);
                TSVector.Add(ref P1, ref output, out output);
            }
        }

        ///// <summary>
        ///// Transform the segment using the specified transformation.
        ///// </summary>
        ///// <param name="s">The segment to transform.</param>
        ///// <param name="transform">the transform to apply.</param>
        ///// <param name="output">Returns the transformed segment.</param>
        //public static void Transform(ref SegmentShape s, ref Transform transform, out SegmentShape output)
        //{
        //    TSVector.Transform(ref s.P1, ref transform.Combined, out output.P1);
        //    TSVector.Transform(ref s.P2, ref transform.Combined, out output.P2);
        //}

        /// <summary>
        /// Intersects two line segments.
        /// </summary>
        /// <param name="sa">The first line segment.</param>
        /// <param name="sb">The second line segment.</param>
        /// <param name="scalarA">Returns a value between 0 and 1 indicating the point of intersection on the first segment.</param>
        /// <param name="scalarB">Returns a value between 0 and 1 indicating the point of intersection on the second segment.</param>
        /// <param name="p">Returns the point of intersection, common to both segments.</param>
        /// <returns>Returns a value indicating whether there was an intersection.</returns>
        public static bool Intersect(ref SegmentShape sa, ref SegmentShape sb, out FP scalarA, out FP scalarB, out TSVector p)
        {
            TSVector pa;
            FP dist;
            SegmentShape.ClosestPoints(ref sa, ref sb, out scalarA, out pa, out scalarB, out p);
            dist = TSVector.Subtract(pa, p).sqrMagnitude;
            return dist < TSMath.Epsilon;
        }

        /// <summary>
        /// Gets the closest two points on two line segments.
        /// </summary>
        /// <remarks>
        /// If the line segments are parallel and overlap in their common direction, then the midpoint of the overlapped portion of line segments
        /// is returned.
        /// </remarks>
        /// <param name="sa">The first line segment.</param>
        /// <param name="sb">The second line segment.</param>
        /// <param name="scalarA">Returns a value between 0 and 1 indicating the position of the closest point on the first segment.</param>
        /// <param name="pa">Returns the closest point on the first segment.</param>
        /// <param name="scalarB">Returns a value between 0 and 1 indicating the position of the closest point on the second segment.</param>
        /// <param name="pb">Returns the closest point on the second segment.</param>
        public static void ClosestPoints(ref SegmentShape sa, ref SegmentShape sb,
            out FP scalarA, out TSVector pa, out FP scalarB, out TSVector pb)
        {
            TSVector d1, d2, r;
            TSVector.Subtract(ref sa.P2, ref sa.P1, out d1);
            TSVector.Subtract(ref sb.P2, ref sb.P1, out d2);
            TSVector.Subtract(ref sa.P1, ref sb.P1, out r);
            FP a, e, f;
            a = TSVector.Dot(ref d1, ref d1);
            e = TSVector.Dot(ref d2, ref d2);
            f = TSVector.Dot(ref d2, ref r);

            if (a < TSMath.Epsilon && e < TSMath.Epsilon)
            {
                // segment a and b are both points
                scalarA = scalarB = FP.Zero;
                pa = sa.P1;
                pb = sb.P1;
                return;
            }

            if (a < TSMath.Epsilon)
            {
                // segment a is a point
                scalarA = FP.Zero;
                scalarB = TSMath.Clamp(f / e, FP.Zero, FP.One);
            }
            else
            {
                FP c = TSVector.Dot(ref d1, ref r);

                if (e < TSMath.Epsilon)
                {
                    // segment b is a point
                    scalarB = FP.Zero;
                    scalarA = TSMath.Clamp(-c / a, FP.Zero, FP.One);
                }
                else
                {
                    FP b = TSVector.Dot(ref d1, ref d2);
                    FP denom = a * e - b * b;

                    if (denom < TSMath.Epsilon)
                    {
                        // segments are parallel
                        FP a1, a2, b1, b2;
                        a1 = TSVector.Dot(ref d2, ref sa.P1);
                        a2 = TSVector.Dot(ref d2, ref sa.P2);
                        b1 = TSVector.Dot(ref d2, ref sb.P1);
                        b2 = TSVector.Dot(ref d2, ref sb.P2);
                        if (a1 <= b1 && a2 <= b1)
                        {
                            // segment A is completely "before" segment B
                            scalarA = a2 > a1 ? FP.One : FP.Zero;
                            scalarB = FP.Zero;
                        }
                        else if (a1 >= b2 && a2 >= b2)
                        {
                            // segment B is completely "before" segment A
                            scalarA = a2 > a1 ? FP.Zero : FP.One;
                            scalarB = FP.One;
                        }
                        else
                        {
                            // segments A and B overlap, use midpoint of shared length
                            if (a1 > a2) { f = a1; a1 = a2; a2 = f; }
                            f = (TSMath.Min(a2, b2) + TSMath.Max(a1, b1)) / 2;
                            scalarB = (f - b1) / e;
                            TSVector.Multiply(ref d2, scalarB, out pb);
                            TSVector.Add(ref sb.P1, ref pb, out pb);
                            sa.ClosestPointTo(ref pb, out scalarA, out pa);
                            return;
                        }
                    }
                    else
                    {
                        // general case
                        scalarA = TSMath.Clamp((b * f - c * e) / denom, FP.Zero, FP.One);
                        scalarB = (b * scalarA + f) / e;
                        if (scalarB < FP.Zero)
                        {
                            scalarB = FP.Zero;
                            scalarA = TSMath.Clamp(-c / a, FP.Zero, FP.One);
                        }
                        else if (scalarB > FP.One)
                        {
                            scalarB = FP.One;
                            scalarA = TSMath.Clamp((b - c) / a, FP.Zero, FP.One);
                        }
                    }
                }
            }
            TSVector.Multiply(ref d1, scalarA, out d1);
            TSVector.Multiply(ref d2, scalarB, out d2);
            TSVector.Add(ref sa.P1, ref d1, out pa);
            TSVector.Add(ref sb.P1, ref d2, out pb);
        }

        public override void SupportMapping(ref TSVector direction, out TSVector result)
        {
            result = TSVector.up; //TODO:
        }

        public void CleanUp()
        {
            //TODO:
        }
    }
}
