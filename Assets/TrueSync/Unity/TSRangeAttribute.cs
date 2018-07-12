using System;
using UnityEngine;

namespace TrueSync
{
    public class TSRangeAttribute : PropertyAttribute
    {
        public readonly float min;
        public readonly float max;
        public readonly float step;

        public TSRangeAttribute(float min, float max, float step = 0.1f)
        {
            this.min = min;
            this.max = max;
            this.step = step;
        }
    }
}