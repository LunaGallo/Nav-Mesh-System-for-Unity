using System.Collections.Generic;
using UnityEngine;

namespace NavMeshSystem {
    namespace Core {
        public class NavPosition_BHV : MonoBehaviour {

            public Navigable_BHV navigableRef = null;
            public int currentRegionIndex = -1;
            public bool autoDetectOnStart = true;

            private void Start() {
                if (autoDetectOnStart) {
                    FindNavigableReference();
                    RedefinePolygonIndex();
                }
            }

            public Vector3 realPosition {
                get {
                    return transform.position;
                }
                set {
                    transform.position = value;
                }
            }
            public Vector3 MeshSnappedPosition() {
                if (navigableRef == null) {
                    throw new System.Exception("NavMeshPosition_BHV has reference to no NavMesh_BHV");
                }
                if (currentRegionIndex > -1) {
                    RedefinePolygonIndex();
                }
                return navigableRef.SnapPointToPoly(realPosition, currentRegionIndex);
            }

            public void FindNavigableReference() {
                navigableRef = Navigable_BHV.GetClosestNavToPoint(realPosition);
            }
            public void RedefinePolygonIndex() {
                currentRegionIndex = navigableRef.GetClosestPolyToPoint(realPosition);
            }
            public bool isPolygonIndexValid {
                get { return (currentRegionIndex > -1); }
            }
            
        }
    }
}