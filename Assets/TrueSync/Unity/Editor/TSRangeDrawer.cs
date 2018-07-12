using UnityEngine;
using UnityEditor;

namespace TrueSync
{
    [CustomPropertyDrawer(typeof(TSRangeAttribute))]
    internal sealed class TSRangeDrawer : PropertyDrawer
    {
        private FP fpValue;

        //
        // Methods
        //
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            var rangeAttribute = (TSRangeAttribute)base.attribute;

            var rawProp = property.FindPropertyRelative("_serializedValue");

            fpValue = EditorGUI.Slider(position, label, fpValue.AsFloat(), rangeAttribute.min, rangeAttribute.max);

            fpValue = (fpValue / rangeAttribute.step) * rangeAttribute.step;

            rawProp.longValue = fpValue.RawValue;
        }
    }
}