using System;

namespace TrueSync
{

    /**
    *  @brief Helpers for 3D physics.
    **/
    public class TSPhysics {

        private static TSRay ray = new TSRay(TSVector.zero, TSVector.zero);
        public static bool Raycast(TSVector rayOrigin, TSVector rayDirection, out TSRaycastHit hit, FP maxDistance, int layerMask = UnityEngine.Physics.DefaultRaycastLayers)
        {
            hit = null;
            if (rayDirection.x == FP.Zero && rayDirection.y == FP.Zero && rayDirection.z == FP.Zero)
                return false;

            ray.origin = rayOrigin;
            ray.direction = rayDirection;

            hit = PhysicsWorldManager.instance.Raycast(ray, maxDistance, layerMask:layerMask);
            if (hit != null)
            {
                if (hit.distance <= maxDistance)
                    return true;
            }
            return false;
        }
    }

}