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
    /// A <see cref="Shape"/> representing a capsule.
    /// </summary>
    public class CapsuleShape : Shape
    {
        internal FP length, radius;

        /// <summary>
        /// Gets or sets the length of the capsule (exclusive the round endcaps).
        /// </summary>
        public FP Length { get { return length; } set { length = value; UpdateShape(); } }

        /// <summary>
        /// Gets or sets the radius of the endcaps.
        /// </summary>
        public FP Radius { get { return radius; } set { radius = value; UpdateShape(); } }

        /// <summary>
        /// Create a new instance of the capsule.
        /// </summary>
        /// <param name="length">The length of the capsule (exclusive the round endcaps).</param>
        /// <param name="radius">The radius of the endcaps.</param>
        public CapsuleShape(FP length,FP radius)
        {
            this.length = length;
            this.radius = radius;
            UpdateShape();
            this.shapeType = ShapeType.Capusle;
        }

        /// <summary>
        /// 
        /// </summary>
        public override void CalculateMassInertia()
        {
            FP rr = radius * radius;
            FP ll = length * length;
            FP massSphere = ( (4 * FP.One) / (3 * FP.One)) * TSMath.Pi * rr * radius;
            FP massCylinder = TSMath.Pi * rr * length;

            mass = massCylinder + massSphere;

            FP massCylinderRR = massCylinder * rr;
            FP massCylinderLL = massCylinder * ll;
            FP massSphereRR = massSphere * rr;
            FP massSphereLL = massSphere * ll;
            FP quarter = (FP.One / (4 * FP.One));
            FP twoOverFive = (2 * FP.One) / (5 * FP.One);

            this.inertia.M11 = quarter * massCylinderRR + (FP.One / (12 * FP.One)) * massCylinderLL + twoOverFive * massSphereRR + quarter * massSphereLL;
            this.inertia.M22 = FP.Half * massCylinderRR + twoOverFive * massSphereRR;
            this.inertia.M33 = this.inertia.M11;

            //this.inertia.M11 = (FP.One / (4 * FP.One)) * mass * radius * radius + (FP.One / (12 * FP.One)) * mass * height * height;
            //this.inertia.M22 = (FP.One / (2 * FP.One)) * mass * radius * radius;
            //this.inertia.M33 = (FP.One / (4 * FP.One)) * mass * radius * radius + (FP.One / (12 * FP.One)) * mass * height * height;
        }


        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        public override void SupportMapping(ref TSVector direction, out TSVector result)
        {
            FP r = FP.Sqrt(direction.x * direction.x + direction.z * direction.z);

            if (FP.Abs(direction.y) > FP.Zero)
            {
                TSVector dir; TSVector.Normalize(ref direction, out dir);
                TSVector.Multiply(ref dir, radius, out result);
                result.y += FP.Sign(direction.y) * FP.Half * length;              
            }
            else if (r > FP.Zero)
            {
                result.x = direction.x / r * radius;
                result.y = FP.Zero;
                result.z = direction.z / r * radius;
            }
            else
            {
                result.x = FP.Zero;
                result.y = FP.Zero;
                result.z = FP.Zero;
            }
        }
    }
}
