using System.Collections.Generic;
using UnityEngine;

namespace NavMeshSystem {
    namespace Core {
        [RequireComponent(typeof(NavPosition_BHV))]
        public class Navigator_BHV : MonoBehaviour {

            private NavPosition_BHV _meshPos = null;
            public NavPosition_BHV meshPos {
                get {
                    if (_meshPos == null) {
                        _meshPos = GetComponent<NavPosition_BHV>();
                    }
                    return _meshPos;
                }
            }
            public Navigable_BHV navMeshRef {
                get {
                    return meshPos.navigableRef;
                }
            }

            public class Path {
                
                public class MeshPath {
                    public Navigable_BHV navMeshRef;
                    public List<int> polyIndexList;
                }
                public List<MeshPath> subPaths;

            }
            public Path currentPath;


        }
    }
}