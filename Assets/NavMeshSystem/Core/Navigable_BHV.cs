using System.Collections.Generic;
using UnityEngine;

namespace NavMeshSystem {
    namespace Core {
        public class Navigable_BHV : MonoBehaviour {
            
            public static List<Navigable_BHV> instanceList = new List<Navigable_BHV>();
            private void Awake() {
                instanceList.Add(this);
            }
            private void OnDestroy() {
                instanceList.Remove(this);
            }
            public static Navigable_BHV GetClosestNavToPoint(Vector3 worldPos) {
                int minIndex = 0;
                if (instanceList.FindMinimum(n => n.MinDistToPoint(worldPos), out minIndex)) {
                    Debug.Log("OK");
                    return instanceList[minIndex];
                }
                return null;
            }
            
            [SerializeField] private Mesh _mesh;
            public Mesh mesh {
                get {
                    return _mesh;
                }
                set {
                    _mesh = Instantiate(value);
                    ReloadMesh();
                }
            }
            public void ReloadMesh() {
                if (mesh != null) {
                    polyList = new List<Poly>();
                    weightList = new List<float>();

                    int vertexCount = mesh.vertexCount;
                    for (int i = 0; i < vertexCount; i++) {
                        if (mesh.IsInnerVertex(i)) {
                            mesh.DissolveVertex(i, 0);
                        }
                    }
                    mesh.SolveTangents();

                    List<Vertex> generalVertexList = new List<Vertex>();
                    for(int i=0; i< vertexCount; i++) {
                        Vertex newVertex = new Vertex();
                        newVertex.pos = mesh.vertices[i];
                        generalVertexList.Add(newVertex);
                    }

                    List<int> tris = new List<int>();
                    tris.AddRange(mesh.triangles);
                    int n = tris.Count;
                    for (int i = 0; i < n; i+=3) {
                        Poly newPoly = new Poly(this);
                        polyList.Add(newPoly);
                        weightList.Add(1f);
                            
                        for (int j = 0; j < 3; j++) {
                            int curID = tris[i+j];
                            Vertex curVertex = generalVertexList[curID];

                            curVertex.owners.Add(newPoly);
                            newPoly.vertexList.Add(curVertex);
                        }
                    }
                }
            }
            public Mesh unfoldedMesh;

            public List<Poly> polyList = new List<Poly>();
            public List<float> weightList = new List<float>(); //Indexed like polyList

            public List<Link> linkList = new List<Link>();
                
            public class Poly {
                public List<Vertex> vertexList = new List<Vertex>();
                public Navigable_BHV ownerNav;

                public Poly(Navigable_BHV ownerNav) {
                    this.ownerNav = ownerNav;
                }

                public Vector3 p0 {
                    get {
                        return vertexList[0].pos;
                    }
                }
                public Vector3 p1 {
                    get {
                        return vertexList[1].pos;
                    }
                }
                public Vector3 p2 {
                    get {
                        return vertexList[2].pos;
                    }
                }

                public Vector3 planeOrigin {
                    get {
                        return vertexList[0].pos;
                    }
                }
                public Vector3 planeNormal {
                    get {
                        return Vector3.Cross(p1 - p0, p2 - p0).normalized;
                    }
                }
                public Vector3 planeTangent {
                    get {
                        return (p1 - p0).normalized;
                    }
                }
                public Vector3 planeBitangent {
                    get {
                        return Vector3.Cross(planeNormal, planeTangent);
                    }
                }

                public float totalArea {
                    get {
                        float result = 0f;
                        for(int i=1; i < vertexList.Count-1; i++) {
                            result += TriArea(vertexList[0].pos, vertexList[i].pos, vertexList[i+1].pos);
                        }
                        return result;
                    }
                }
                private float TriArea(Vector3 v0, Vector3 v1, Vector3 v2) {
                    return Vector3.Cross(v1 - v0, v2 - v0).magnitude / 2f;
                }
                public bool hasNormal {
                    get {
                        return totalArea > 0f;
                    }
                }
                public Vector3 UpFromPortal(Vertex leftVertex, Vertex rightVertex) {
                    Vector3 portalVector = leftVertex.pos - rightVertex.pos;
                    Vertex thirdVertex = vertexList.Find(v => Vector3.Dot(v.pos - rightVertex.pos, Quaternion.AngleAxis(90f, planeNormal) * portalVector) != 0f);
                    return Vector3.Cross(portalVector, thirdVertex.pos - rightVertex.pos);
                }

                public Vector3 center {
                    get {
                        return vertexList.SumElement<Vector3, Vertex>((v,e) => v + e.pos);
                    }
                }

                public Vector3 Projection(Vector3 point) {
                    return Vector3.ProjectOnPlane(point - planeOrigin, planeNormal) + planeOrigin;
                }

                public bool ContainsPoint(Vector3 point) {
                    for (int i = 1; i < vertexList.Count - 1; i++) {
                        if (IsPointInsideTriangle(point, p0, vertexList[i].pos, vertexList[i+1].pos)) {
                            return true;
                        }
                    }
                    return false;
                }
                public Vertex[] NeighborVertices(Vertex cur) {
                    Vertex[] neighbors = new Vertex[2];
                    int curIndex = vertexList.IndexOf(cur);
                    neighbors[0] = vertexList[curIndex - 1];
                    neighbors[1] = vertexList[curIndex + 1];
                    return neighbors;
                }
                public List<Vertex> GetPortalVertices() {
                    return vertexList.FindAll(v => v.isPortal);
                }
                public List<Vertex> OtherVertices(Vertex vertex) {
                    List<Vertex> result = new List<Vertex>();
                    result.AddRange(vertexList);
                    result.Remove(vertex);
                    return result;
                }
                public List<Vertex> OtherPortalVertices(Vertex vertex) {
                    List<Vertex> result = new List<Vertex>();
                    result.AddRange(vertexList.FindAll(v => v.isPortal));
                    result.Remove(vertex);
                    return result;
                }
                public List<Poly> NeighborPolys() {
                    List<Poly> neighborList = new List<Poly>();
                    vertexList.FindAll(v => v.isPortal).ForEach(v => neighborList.AddRangeIfDoesntContain(v.GetOtherOwners(this)));
                    return neighborList;
                }
                public List<Vertex> PortalTo(Poly other) {
                    return vertexList.FindAll(v => v.GetOtherOwners(this).Contains(other));
                }
                public Vertex ClosestVertexToPoint(Vector3 point) {
                    int minIndex = 0;
                    vertexList.FindMinimum(v => v.SqrDistanceToPoint(point), out minIndex);
                    return vertexList[minIndex];
                }
                public Vertex ClosestPortalVertexToPoint(Vector3 point) {
                    int minIndex = 0;
                    vertexList.FindAll(v => v.isPortal).FindMinimum(v => v.SqrDistanceToPoint(point), out minIndex);
                    return vertexList[minIndex];
                }
                private static bool IsPointInsideTriangle(Vector3 point, Vector3 v0, Vector3 v1, Vector3 v2) {
                    Vector3 u = v1 - v0;
                    Vector3 v = v2 - v0;
                    Vector3 normal = Vector3.Cross(u, v).normalized;
                    Vector3 p = point - v0;
                    float x = Vector3.Dot(p, Vector3.Cross(normal, u).normalized) / u.magnitude;
                    float y = Vector3.Dot(p, Vector3.Cross(v, normal).normalized) / v.magnitude;
                    return ((x >= 0) && (y >= 0) && (x + y <= 0.5f));
                }

            }
            public class Vertex {
                public Vector3 pos;
                [System.NonSerialized]
                public List<Poly> owners = new List<Poly>();
                public Navigable_BHV ownerNav {
                    get {
                        return owners[0].ownerNav;
                    }
                }
                
                public List<Vertex> linkedVertices {
                    get {
                        List<Vertex> result = new List<Vertex>();
                        owners.ForEach(p => result.AddRangeIfDoesntContain(p.OtherPortalVertices(this)));
                        return result;
                    }
                }
                public List<Poly> GetOtherOwners(Poly p) {
                    List<Poly> result = new List<Poly>();
                    result.AddRange(owners);
                    result.Remove(p);
                    return result;
                }
                public bool isPortal {
                    get {
                        return (owners.Count > 1);
                    }
                }
                public float SqrDistanceToPoint(Vector3 localPoint) {
                    return (localPoint - pos).sqrMagnitude;
                }
            }
            public class Link {
                public Vertex originVertex;
                public Vertex finishVertex;
                    
                public Navigable_BHV originNav = null;
                public Navigable_BHV finishNav = null;

                public Vertex originSecondVertex;
                public Vertex finishSecondVertex;
                    
                public bool forwardOnly = false;
                [Range(1f,float.MaxValue)] public float weight = 1f;
                public float estimateCost {
                    get {
                        return weight * (GetGlobalPoint(finishNav) - GetGlobalPoint(originNav)).magnitude;
                    }
                }

                public Vector3 originVertexInWorld { get { return originNav.transform.TransformVector(originVertex.pos); } }
                public Vector3 finishVertexInWorld { get { return finishNav.transform.TransformVector(finishVertex.pos); } }
                public Vector3 originSecondVertexInWorld { get { return originNav.transform.TransformVector(originSecondVertex.pos); } }
                public Vector3 finishSecondVertexInWorld { get { return finishNav.transform.TransformVector(finishSecondVertex.pos); } }

                public Vertex GetVertex(Navigable_BHV nav) {
                    if (nav == originNav) {
                        return originVertex;
                    }
                    else if (nav == finishNav) {
                        return finishVertex;
                    }
                    else {
                        throw new System.Exception("Trying to get the vertex of a navmesh link through the wrong navigable");
                    }
                }
                public Vertex GetSecondVertex(Navigable_BHV nav) {
                    if (nav == originNav) {
                        return originSecondVertex;
                    }
                    else if (nav == finishNav) {
                        return finishSecondVertex;
                    }
                    else {
                        throw new System.Exception("Trying to get the vertex of a navmesh link through the wrong navigable");
                    }
                }
                public Vector3 GetLocalPoint(Navigable_BHV nav) {
                    return GetVertex(nav).pos;
                }
                public Vector3 GetGlobalPoint(Navigable_BHV nav) {
                    return nav.transform.TransformVector(GetVertex(nav).pos);
                }
                public Vector3 GetDimension(Navigable_BHV nav) {
                    return GetSecondVertex(nav).pos - GetVertex(nav).pos;
                }

                public Navigable_BHV GetOtherNav(Navigable_BHV nav) {
                    if (nav == originNav) {
                        return finishNav;
                    }
                    else if (nav == finishNav) {
                        return originNav;
                    }
                    else {
                        throw new System.Exception("Trying to get the other end of a navmesh link through the wrong navigable");
                    }
                }
                public Vertex GetOtherVertex(Navigable_BHV nav) {
                    return GetVertex(GetOtherNav(nav));
                }
                public Vertex GetOtherSecondVertex(Navigable_BHV nav) {
                    return GetSecondVertex(GetOtherNav(nav));
                }
                public Vector3 GetOtherLocalPoint(Navigable_BHV nav) {
                    return GetLocalPoint(GetOtherNav(nav));
                }
                public Vector3 GetOtherGlobalPoint(Navigable_BHV nav) {
                    return GetGlobalPoint(GetOtherNav(nav));
                }
                public Vector3 GetOtherDimension(Navigable_BHV nav) {
                    return GetDimension(GetOtherNav(nav));
                }

                public Vector3 normal {
                    get {
                        Vector3 originToFinish = finishVertexInWorld - originVertexInWorld;
                        Vector3 tangent = Vector3.right;
                        if (finishSecondVertex != null) {
                            tangent = finishSecondVertexInWorld - originVertexInWorld;
                        }
                        else if (originSecondVertex != null) {
                            tangent = originSecondVertexInWorld - originVertexInWorld;
                        }
                        else if (originToFinish != Vector3.up) {
                            tangent = Vector3.Cross(Vector3.up, originToFinish);
                        }
                        return Vector3.Cross(originToFinish, originSecondVertexInWorld - originVertexInWorld).normalized;
                    }
                }
                public Vector3 UpFromPortal(Vertex leftVertex, Vertex rightVertex) {
                    if ((leftVertex == originVertex) || (rightVertex == finishVertex)){
                        return normal;
                    }
                    else {
                        return -normal;
                    }
                }
            }
            
            public int GetClosestPolyToPoint(Vector3 worldPos) {
                Vector3 localPos = transform.InverseTransformVector(worldPos);
                int minIndex = -1;
                polyList.FindAll(p => p.ContainsPoint(localPos)).FindMinimum(p => ((p.Projection(localPos)) - localPos).sqrMagnitude, out minIndex);
                return minIndex;
            }
            public Vector3 SnapPointToPoly(Vector3 worldPos, int polyIndex) {
                Vector3 localPos = transform.InverseTransformVector(worldPos);
                Vector3 result = localPos;
                if ((polyIndex >= 0) && (polyIndex < polyList.Count)) {
                    Poly poly = polyList[polyIndex];
                    if (poly.ContainsPoint(localPos)) {
                        result = poly.Projection(localPos);
                    }
                }
                return transform.TransformVector(result);
            }
            public float MinDistToPoint(Vector3 worldPos) {
                Vector3 localPos = transform.InverseTransformVector(worldPos);
                float minDist = float.MaxValue;
                polyList.FindAll(p => p.ContainsPoint(localPos)).FindMinimum(p => ((p.Projection(localPos)) - localPos).sqrMagnitude, out minDist);
                return minDist;
            }

            public List<Vertex> AStarPathFinding(int startPolyID, Vector3 startPoint, int endPolyID, Vector3 endPoint) {
                if (startPolyID < 0) { return null; }
                if (endPolyID < 0) { return null; }

                List<Vertex> openVertexList = new List<Vertex>();
                List<float> openVertexTreeValue = new List<float>();
                List<Vertex> openVertexTreeParent = new List<Vertex>();
                List<bool> closedVertexList = new List<bool>();

                Poly startPoly = polyList[startPolyID];
                Poly endPoly = polyList[endPolyID];

                foreach (Vertex v in startPoly.GetPortalVertices()) {
                    openVertexList.Add(v);
                    openVertexTreeValue.Add(Vector3.Distance(v.pos, startPoint) * weightList[startPolyID]);
                    openVertexTreeParent.Add(null);
                    closedVertexList.Add(false);

                }

                int minIndex = -1;
                float minValue = float.MaxValue;
                for (int i=0; i<openVertexList.Count; i++) {
                    float curValue = openVertexTreeValue[i] + Vector3.Distance(openVertexList[i].pos, endPoint);
                    if (closedVertexList[i] == false) {
                        if (minValue > curValue) {
                            minValue = curValue;
                            minIndex = i;
                        }
                    }
                }
                if (minIndex < 0) {
                    return null;
                }
                int curIndex = minIndex;
                Vertex curVertex = openVertexList[minIndex];
                closedVertexList[minIndex] = true;
                    

                while (!curVertex.owners.Contains(endPoly)) {
                    foreach (Vertex linkedVertex in curVertex.linkedVertices) {
                        int linkedIndex = openVertexList.IndexOf(linkedVertex);
                        float linkWeight = 0f;
                        List<Poly> sharedOwners = curVertex.owners.FindAll(p => linkedVertex.owners.Contains(p));
                        sharedOwners.ForEach(p => linkWeight += (weightList[polyList.IndexOf(p)]) / sharedOwners.Count);
                        float newPathValue = openVertexTreeValue[curIndex] + (Vector3.Distance(curVertex.pos, linkedVertex.pos) * linkWeight);
                        if (linkedIndex >= 0) {
                            if (closedVertexList[linkedIndex] == false) {
                                if (openVertexTreeValue[linkedIndex] > newPathValue) {
                                    openVertexTreeValue[linkedIndex] = newPathValue;
                                    openVertexTreeParent[linkedIndex] = curVertex;
                                }
                            }
                        }
                        else {
                            openVertexList.Add(linkedVertex);
                            openVertexTreeValue.Add(newPathValue);
                            openVertexTreeParent.Add(curVertex);
                            closedVertexList.Add(false);
                        }
                    }

                    minIndex = -1;
                    minValue = float.MaxValue;
                    for (int i = 0; i < openVertexList.Count; i++) {
                        float curValue = openVertexTreeValue[i] + Vector3.Distance(openVertexList[i].pos, endPoint);
                        if (closedVertexList[i] == false) {
                            if (minValue > curValue) {
                                minValue = curValue;
                                minIndex = i;
                            }
                        }
                    }
                    if (minIndex < 0) {
                        return null;
                    }
                    curIndex = minIndex;
                    curVertex = openVertexList[minIndex];
                    closedVertexList[minIndex] = true;
                }

                List<Vertex> result = new List<Vertex>();
                while (curVertex != null) {
                    result.Add(curVertex);
                    curVertex = openVertexTreeParent[openVertexList.IndexOf(curVertex)];
                }
                result.Reverse();
                return result;
            }
                
            public Vector3 SnapPointToMesh(Vector3 worldPos) {
                return SnapPointToPoly(worldPos, GetClosestPolyToPoint(worldPos));
            }

            public void SetRegionWeight(int regionId, float value) {
                weightList[regionId] = value;
            }
            public float GetRegionWeigth(int regionId) {
                return weightList[regionId];
            }

            public void AddLinkToMesh(Link newLink) {
                linkList.Add(newLink);
            }
            public void RemoveLinkFromMesh(Link link) {
                linkList.Remove(link);
            }

            public List<MeshPath> pathCache = new List<MeshPath>();
            public class Path {
                public Path(MeshPath singlePath) {
                    subPaths = new List<MeshPath>();
                    subPaths.Add(singlePath);
                    CalculateWaypoints();
                }
                public Path(List<MeshPath> allPaths) {
                    subPaths = allPaths;
                    CalculateWaypoints();
                }

                public List<MeshPath> subPaths = new List<MeshPath>();
                public float completeLength {
                    get {
                        return subPaths.SumElement<float,MeshPath>((v,e) => v + e.length);
                    }
                }
                public float completeCost {
                    get {
                        return subPaths.SumElement<float, MeshPath>((v, e) => v + e.cost);
                    }
                }

                private void CalculateWaypoints() {
                    if (subPaths.Count == 0) {
                        return;
                    }
                    
                    List<Vector2> leftPoints = new List<Vector2>();
                    List<Vector2> rightPoints = new List<Vector2>();

                    Vector3 curOrigin3 = subPaths[0].startPoint;
                    Poly curPoly = subPaths[0].navRef.polyList[subPaths[0].polyIndexList[0]];
                    Vector3 curRight = curPoly.planeBitangent;
                    Vector3 curUp = curPoly.planeTangent;
                    Vector3 curNormal = curPoly.planeNormal;
                    Vector2 curOrigin2 = Vector2.zero;

                    for (int subIndex = 0; subIndex<subPaths.Count; subIndex++) {
                        MeshPath subPath = subPaths[subIndex];
                        if ((subIndex > 0) && (subPath.startLink != null)) {
                            Poly firstPoly = subPath.navRef.polyList[0];
                            Link link = subPath.startLink;
                            Vertex secondVertex = link.GetSecondVertex(subPath.navRef);
                            List<Vertex> portalVertices = new List<Vertex>();
                            portalVertices.Add(link.GetVertex(subPath.navRef));
                            int newLeftIndex = 0;
                            int newRightIndex = 0;
                            if (secondVertex != null) {
                                portalVertices.Add(secondVertex);
                                Vector3 referencePoint = curOrigin3;
                                if (curPoly.hasNormal) {
                                    referencePoint = curPoly.center;
                                }
                                if (IsPortalInOrder(portalVertices, curNormal, referencePoint)) {
                                    newRightIndex = portalVertices.Count - 1;
                                }
                                else {
                                    newLeftIndex = portalVertices.Count - 1;
                                }
                            }
                            Vector3 newLeft3D = subPath.navRef.transform.TransformVector(portalVertices[newLeftIndex].pos);
                            Vector3 newRight3D = subPath.navRef.transform.TransformVector(portalVertices[newRightIndex].pos);
                            Vector2 newLeft2D = UnwrapVector(curOrigin2, curOrigin3, curUp, curRight, newLeft3D);
                            Vector2 newRight2D = UnwrapVector(curOrigin2, curOrigin3, curUp, curRight, newRight3D);
                            leftPoints.Add(newLeft2D);
                            rightPoints.Add(newRight2D);
                            curOrigin2 = newRight2D;
                            curOrigin3 = newRight3D;
                            Vector3 nextNormal = link.UpFromPortal(portalVertices[newLeftIndex], portalVertices[newRightIndex]);
                            if (nextNormal.sqrMagnitude != 0f) {
                                Quaternion egdeRotation = Quaternion.FromToRotation(curNormal, nextNormal);
                                curRight = egdeRotation * curRight;
                                curUp = egdeRotation * curUp;
                                curNormal = nextNormal;
                            }
                        }
                        for (int i=0; i<subPath.polyIndexList.Count-1; i++) {
                            int curPolyIndex = subPath.polyIndexList[i];
                            int nextPolyIndex = subPath.polyIndexList[i+1];
                            curPoly = subPath.navRef.polyList[curPolyIndex];
                            Poly nextPoly = subPath.navRef.polyList[nextPolyIndex];
                            List<Vertex> portalVertices = new List<Vertex>();
                            portalVertices.AddRange(curPoly.PortalTo(nextPoly));
                            int newLeftIndex = 0;
                            int newRightIndex = 0;
                            if (portalVertices.Count > 1) {
                                Vector3 referencePoint = curOrigin3;
                                if (curPoly.hasNormal) {
                                    referencePoint = curPoly.center;
                                }
                                if (IsPortalInOrder(portalVertices, curNormal, referencePoint)) {
                                    newRightIndex = portalVertices.Count - 1;
                                }
                                else {
                                    newLeftIndex = portalVertices.Count - 1;
                                }
                            }
                            Vector3 newLeft3D = subPath.navRef.transform.TransformVector(portalVertices[newLeftIndex].pos);
                            Vector3 newRight3D = subPath.navRef.transform.TransformVector(portalVertices[newRightIndex].pos);
                            Vector2 newLeft2D = UnwrapVector(curOrigin2, curOrigin3, curUp, curRight, newLeft3D);
                            Vector2 newRight2D = UnwrapVector(curOrigin2, curOrigin3, curUp, curRight, newRight3D);
                            leftPoints.Add(newLeft2D);
                            rightPoints.Add(newRight2D);
                            curOrigin2 = newRight2D;
                            curOrigin3 = newRight3D;
                            Vector3 nextNormal = subPath.navRef.transform.TransformDirection(nextPoly.UpFromPortal(portalVertices[newLeftIndex], portalVertices[newRightIndex]));
                            if (nextNormal.sqrMagnitude != 0f) {
                                Quaternion egdeRotation = Quaternion.FromToRotation(curNormal, nextNormal);
                                curRight = egdeRotation * curRight;
                                curUp = egdeRotation * curUp;
                                curNormal = nextNormal;
                            }
                        }
                        if ((subIndex + 1 < subPaths.Count) && (subPath.endLink != null)){
                            Link link = subPath.endLink;
                            Vertex secondVertex = link.GetSecondVertex(subPath.navRef);
                            List<Vertex> portalVertices = new List<Vertex>();
                            portalVertices.Add(link.GetVertex(subPath.navRef));
                            int newLeftIndex = 0;
                            int newRightIndex = 0;
                            if (secondVertex != null) {
                                portalVertices.Add(secondVertex);
                                Vector3 referencePoint = curOrigin3;
                                if (curPoly.hasNormal) {
                                    referencePoint = curPoly.center;
                                }
                                if (IsPortalInOrder(portalVertices, curNormal, referencePoint)) {
                                    newRightIndex = portalVertices.Count - 1;
                                }
                                else {
                                    newLeftIndex = portalVertices.Count - 1;
                                }
                            }
                            Vector3 newLeft3D = subPath.navRef.transform.TransformVector(portalVertices[newLeftIndex].pos);
                            Vector3 newRight3D = subPath.navRef.transform.TransformVector(portalVertices[newRightIndex].pos);
                            Vector2 newLeft2D = UnwrapVector(curOrigin2, curOrigin3, curUp, curRight, newLeft3D);
                            Vector2 newRight2D = UnwrapVector(curOrigin2, curOrigin3, curUp, curRight, newRight3D);
                            leftPoints.Add(newLeft2D);
                            rightPoints.Add(newRight2D);
                            curOrigin2 = newRight2D;
                            curOrigin3 = newRight3D;
                            Vector3 nextNormal = link.UpFromPortal(portalVertices[newLeftIndex], portalVertices[newRightIndex]);
                            if (nextNormal.sqrMagnitude != 0f) {
                                Quaternion egdeRotation = Quaternion.FromToRotation(curNormal, nextNormal);
                                curRight = egdeRotation * curRight;
                                curUp = egdeRotation * curUp;
                                curNormal = nextNormal;
                            }
                        }
                    }

                    List<Vector2> allWaypoints = SimpleStupidFunnel(leftPoints, rightPoints, subPaths[subPaths.Count-1].endPoint);

                    curOrigin3 = subPaths[0].startPoint;
                    curPoly = subPaths[0].navRef.polyList[subPaths[0].polyIndexList[0]];
                    curRight = curPoly.planeBitangent;
                    curUp = curPoly.planeTangent;
                    curNormal = curPoly.planeNormal;
                    curOrigin2 = Vector2.zero;
                    int curPointIndex = 0;

                    for (int subIndex = 0; subIndex < subPaths.Count; subIndex++) {
                        MeshPath subPath = subPaths[subIndex];
                        if ((subIndex > 0) && (subPath.startLink != null)) {
                            Poly firstPoly = subPath.navRef.polyList[0];
                            Link link = subPath.startLink;
                            Vertex secondVertex = link.GetSecondVertex(subPath.navRef);
                            List<Vertex> portalVertices = new List<Vertex>();
                            portalVertices.Add(link.GetVertex(subPath.navRef));
                            int newLeftIndex = 0;
                            int newRightIndex = 0;
                            if (secondVertex != null) {
                                portalVertices.Add(secondVertex);
                                Vector3 referencePoint = curOrigin3;
                                if (curPoly.hasNormal) {
                                    referencePoint = curPoly.center;
                                }
                                if (IsPortalInOrder(portalVertices, curNormal, referencePoint)) {
                                    newRightIndex = portalVertices.Count - 1;
                                }
                                else {
                                    newLeftIndex = portalVertices.Count - 1;
                                }
                            }
                            Vector3 newRight3D = subPath.navRef.transform.TransformVector(portalVertices[newRightIndex].pos);
                            Vector2 newRight2D = UnwrapVector(curOrigin2, curOrigin3, curUp, curRight, newRight3D);
                            subPath.startPoint = WrapVector(curOrigin2, curOrigin3, curUp, curRight, allWaypoints[curPointIndex]);
                            curPointIndex++;
                            curOrigin2 = newRight2D;
                            curOrigin3 = newRight3D;
                            Vector3 nextNormal = link.UpFromPortal(portalVertices[newLeftIndex], portalVertices[newRightIndex]);
                            if (nextNormal.sqrMagnitude != 0f) {
                                Quaternion egdeRotation = Quaternion.FromToRotation(curNormal, nextNormal);
                                curRight = egdeRotation * curRight;
                                curUp = egdeRotation * curUp;
                                curNormal = nextNormal;
                            }
                        }
                        for (int i = 0; i < subPath.polyIndexList.Count - 1; i++) {
                            int curPolyIndex = subPath.polyIndexList[i];
                            int nextPolyIndex = subPath.polyIndexList[i + 1];
                            curPoly = subPath.navRef.polyList[curPolyIndex];
                            Poly nextPoly = subPath.navRef.polyList[nextPolyIndex];
                            List<Vertex> portalVertices = new List<Vertex>();
                            portalVertices.AddRange(curPoly.PortalTo(nextPoly));
                            int newLeftIndex = 0;
                            int newRightIndex = 0;
                            if (portalVertices.Count > 1) {
                                Vector3 referencePoint = curOrigin3;
                                if (curPoly.hasNormal) {
                                    referencePoint = curPoly.center;
                                }
                                if (IsPortalInOrder(portalVertices, curNormal, referencePoint)) {
                                    newRightIndex = portalVertices.Count - 1;
                                }
                                else {
                                    newLeftIndex = portalVertices.Count - 1;
                                }
                            }
                            Vector3 newRight3D = subPath.navRef.transform.TransformVector(portalVertices[newRightIndex].pos);
                            Vector2 newRight2D = UnwrapVector(curOrigin2, curOrigin3, curUp, curRight, newRight3D);
                            subPath.waypoints.Add(WrapVector(curOrigin2, curOrigin3, curUp, curRight, allWaypoints[curPointIndex]));
                            curPointIndex++;
                            curOrigin2 = newRight2D;
                            curOrigin3 = newRight3D;
                            Vector3 nextNormal = subPath.navRef.transform.TransformDirection(nextPoly.UpFromPortal(portalVertices[newLeftIndex], portalVertices[newRightIndex]));
                            if (nextNormal.sqrMagnitude != 0f) {
                                Quaternion egdeRotation = Quaternion.FromToRotation(curNormal, nextNormal);
                                curRight = egdeRotation * curRight;
                                curUp = egdeRotation * curUp;
                                curNormal = nextNormal;
                            }
                        }
                        if ((subIndex + 1 < subPaths.Count) && (subPath.endLink != null)) {
                            Link link = subPath.endLink;
                            Vertex secondVertex = link.GetSecondVertex(subPath.navRef);
                            List<Vertex> portalVertices = new List<Vertex>();
                            portalVertices.Add(link.GetVertex(subPath.navRef));
                            int newLeftIndex = 0;
                            int newRightIndex = 0;
                            if (secondVertex != null) {
                                portalVertices.Add(secondVertex);
                                Vector3 referencePoint = curOrigin3;
                                if (curPoly.hasNormal) {
                                    referencePoint = curPoly.center;
                                }
                                if (IsPortalInOrder(portalVertices, curNormal, referencePoint)) {
                                    newRightIndex = portalVertices.Count - 1;
                                }
                                else {
                                    newLeftIndex = portalVertices.Count - 1;
                                }
                            }
                            Vector3 newRight3D = subPath.navRef.transform.TransformVector(portalVertices[newRightIndex].pos);
                            Vector2 newRight2D = UnwrapVector(curOrigin2, curOrigin3, curUp, curRight, newRight3D);
                            subPath.endPoint = WrapVector(curOrigin2, curOrigin3, curUp, curRight, allWaypoints[curPointIndex]);
                            curPointIndex++;
                            curOrigin2 = newRight2D;
                            curOrigin3 = newRight3D;
                            Vector3 nextNormal = link.UpFromPortal(portalVertices[newLeftIndex], portalVertices[newRightIndex]);
                            if (nextNormal.sqrMagnitude != 0f) {
                                Quaternion egdeRotation = Quaternion.FromToRotation(curNormal, nextNormal);
                                curRight = egdeRotation * curRight;
                                curUp = egdeRotation * curUp;
                                curNormal = nextNormal;
                            }
                        }
                    }

                }
                private Vector2 UnwrapVector(Vector2 origin2, Vector3 origin3, Vector3 up, Vector3 right, Vector3 vector) {
                    Vector3 relativeVector = vector - origin3;
                    return origin2 + new Vector2(Vector3.Dot(relativeVector, right), Vector3.Dot(relativeVector, up));
                }
                private Vector3 WrapVector(Vector2 origin2, Vector3 origin3, Vector3 up, Vector3 right, Vector2 vector) {
                    Vector2 relativeVector = vector - origin2;
                    return origin3 + (up.normalized * relativeVector.y) + (right.normalized * relativeVector.x);
                }
                private bool IsPortalInOrder(List<Vertex> portalVertices, Vector3 normal, Vector3 referencePoint) {
                    Vector3 zeroToOne = portalVertices[portalVertices.Count-1].pos - portalVertices[0].pos;
                    Vector3 relativeReferencePoint = referencePoint - portalVertices[0].pos;
                    return (Vector3.Dot(relativeReferencePoint, Vector3.Cross(normal, zeroToOne).normalized) > 0f);
                }
            }
            public class MeshPath {
                public Navigable_BHV navRef;
                public List<int> polyIndexList;

                public Vector3 startPoint;
                public List<Vector3> waypoints;
                public Vector3 endPoint;
                public List<Vector3> allPoints {
                    get {
                        List<Vector3> result = new List<Vector3>();
                        result.Add(startPoint);
                        result.AddRange(waypoints);
                        result.Add(endPoint);
                        return result;
                    }
                    set {
                        startPoint = value[0];
                        endPoint = value[value.Count - 1];
                        waypoints = value;
                        waypoints.RemoveAt(0);
                        waypoints.RemoveAt(waypoints.Count - 1);
                    }
                }

                public Link startLink = null;
                public Link endLink = null;

                public float length {
                    get {
                        return allPoints.SumElementPairs<float,Vector3>((v, f, s) => v + Vector3.Distance(f, s));
                    }
                }
                public float cost {
                    get {
                        float sum = 0f;
                        List<Vector3> pointList = allPoints;
                        for (int i=0; i<polyIndexList.Count; i++) {
                            sum += Vector3.Distance(pointList[i], pointList[i+1]) * navRef.weightList[polyIndexList[i]];
                        }
                        return sum;
                    }
                }
                
                public MeshPath(Navigable_BHV nav, List<Vector3> allPoints) {
                    navRef = nav;
                    this.allPoints = allPoints;
                }
                public MeshPath(Navigable_BHV nav, Vector3 startPoint, int startPoly, Vector3 endPoint, int endPoly, List<Vertex> vertexList) {
                    navRef = nav;
                    this.startPoint = startPoint;
                    this.endPoint = endPoint;
                    if (vertexList != null) {
                        polyIndexList = VertexToPolyPath(vertexList, startPoly, endPoly);
                    }
                }
                public MeshPath(Navigable_BHV nav, Vector3 startPoint, Vector3 endPoint, List<int> polyList) {
                    navRef = nav;
                    this.startPoint = startPoint;
                    this.endPoint = endPoint;
                    polyIndexList = polyList;
                }

                public static MeshPath Reversed(MeshPath original) {
                    MeshPath newPath = new MeshPath(original.navRef, original.allPoints);
                    newPath.polyIndexList = new List<int>();
                    newPath.polyIndexList.AddRange(original.polyIndexList);
                    newPath.polyIndexList.Reverse();
                    newPath.waypoints.Reverse();
                    newPath.startPoint = original.endPoint;
                    newPath.endPoint = original.startPoint;
                    newPath.startLink = original.endLink;
                    newPath.endLink = original.startLink;
                    return newPath;
                }

                private List<int> VertexToPolyPath(List<Vertex> vertexList, int startPoly, int endPoly) {
                    List<List<int>> polyGroups = new List<List<int>>();
                    for (int i = 0; i < vertexList.Count - 1; i++) {
                        Vertex curVertex = vertexList[i];
                        Vertex nextVertex = vertexList[i + 1];
                        List<int> curGroup = new List<int>();
                        curVertex.owners.FindAll(p => nextVertex.owners.Contains(p)).ForEach(p => curGroup.Add(navRef.polyList.IndexOf(p)));
                        polyGroups.Add(curGroup);
                    }

                    List<int> result = new List<int>();
                    int curPoly = startPoly;
                    for (int i = 0; i < polyGroups.Count; i++) {
                        List<int> curGroup = polyGroups[i];
                        Vertex curVertex = vertexList[i];
                        result.AddRange(FindWayArroundVertexFromPoly(curVertex, curGroup, curPoly, out curPoly));
                    }
                    result.AddIfDoesntContain(endPoly);
                    return result;
                }
                private List<int> FindWayArroundVertexFromPoly(Vertex vertex, List<int> goal, int startPoly, out int curPoly) {
                    List<int> way = new List<int>();
                    List<int> discoveredPolyList = new List<int>();
                    List<int> discoveredPolyParentList = new List<int>();
                    List<bool> closedPolyList = new List<bool>();
                    discoveredPolyList.Add(startPoly);
                    discoveredPolyParentList.Add(-1);
                    closedPolyList.Add(false);
                    curPoly = startPoly;
                    while (discoveredPolyList.FindIndex(p => goal.Contains(p)) == -1) {
                        List<Poly> validNeighbors = navRef.polyList[curPoly].NeighborPolys().FindAll(p => p.vertexList.Contains(vertex));
                        foreach (Poly p in validNeighbors) {
                            int index = navRef.polyList.IndexOf(p);
                            if (!discoveredPolyList.Contains(index)) {
                                discoveredPolyList.Add(index);
                                discoveredPolyParentList.Add(curPoly);
                                closedPolyList.Add(false);
                            }
                        }
                        closedPolyList[curPoly] = true;
                        curPoly = -1;
                        for (int i = 0; i < discoveredPolyList.Count; i++) {
                            if (closedPolyList[i] == false) {
                                curPoly = i;
                                break;
                            }
                        }
                        if (curPoly < 0) {
                            return null;
                        }
                    }
                    way.Add(discoveredPolyParentList[curPoly]);
                    while (discoveredPolyParentList[discoveredPolyList.IndexOf(way[0])] != -1) {
                        way.Insert(0, discoveredPolyParentList[discoveredPolyList.IndexOf(way[0])]);
                    }
                    return way;
                }
            }
            public static List<Vector2> SimpleStupidFunnel(List<Vector2> leftPoints, List<Vector2> rightPoints, Vector2 finish) {
                List<Vector2> result = new List<Vector2>();
                result.Add(Vector2.zero);
                
                int l = 0;
                int r = 0;
                Vector2 currentPoint = Vector2.zero;
                int n = rightPoints.Count;

                while ((l < n) && (r < n)) {
                    Vector2 currentLeft = leftPoints[l];
                    Vector2 currentRight = rightPoints[r];

                    Vector2 nextPoint = finish;
                    if (l+1 < n) {
                        nextPoint = leftPoints[l + 1];
                    }
                    int leftResult = EvaluateFunnel(currentPoint, currentLeft, currentRight, nextPoint);
                    bool leftChanged = false;
                    if (leftResult == 1) {
                        leftChanged = true;
                        l++;
                    }

                    nextPoint = finish;
                    if (r + 1 < n) {
                        nextPoint = rightPoints[r + 1];
                    }
                    int rightResult = EvaluateFunnel(currentPoint, currentLeft, currentRight, nextPoint);
                    bool rightChanged = false;
                    if (rightResult == 1) {
                        rightChanged = true;
                        r++;
                    }

                    if ((!leftChanged) && (!rightChanged)) {
                        if ((leftResult == 0) && (rightResult == 0)) {
                            for (int i=result.Count-1; i<=l; i++) {
                                result.Add(LineIntersection(currentPoint, currentLeft - currentPoint, leftPoints[i], rightPoints[i] - leftPoints[i]));
                            }
                            l++;
                            r++;
                            currentPoint = currentLeft;
                        }
                        else if ((leftResult == 2) && (rightResult == 2)) {
                            for (int i = result.Count - 1; i <= l; i++) {
                                result.Add(LineIntersection(currentPoint, currentRight - currentPoint, leftPoints[i], rightPoints[i] - leftPoints[i]));
                            }
                            l++;
                            r++;
                            currentPoint = currentRight;
                        }
                        else {
                            throw new System.Exception("SSF found wrong funnel.");
                        }
                    }

                }
                for (int i = result.Count - 1; i < n; i++) {
                    result.Add(LineIntersection(currentPoint, finish - currentPoint, leftPoints[i], rightPoints[i] - leftPoints[i]));
                }
                result.Add(finish);
                return result;
            }
            private static int EvaluateFunnel(Vector2 origin, Vector2 left, Vector2 right, Vector2 point) {
                Vector2 relativePoint = point - origin;
                Vector2 leftNormal = (new Vector2(left.y, -left.x)).normalized;
                Vector2 rightNormal = (new Vector2(-right.y, right.x)).normalized;
                float leftBorderValue = Vector2.Dot(relativePoint, leftNormal);
                float rightBorderValue = Vector2.Dot(relativePoint, rightNormal);
                if (leftBorderValue >= 0) {
                    if (rightBorderValue >= 0) {
                        return 1;
                    }
                    else {
                        return 2;
                    }
                }
                else {
                    if (rightBorderValue >= 0) {
                        return 0;
                    }
                    else {
                        return -1;
                    }
                }
            }
            private static Vector2 LineIntersection(Vector2 origin1, Vector2 direction1, Vector2 origin2, Vector2 direction2) {
                Quaternion dir1Rotation = Quaternion.FromToRotation(direction1, new Vector3(1,0));
                Vector2 relativeOrigin2 = dir1Rotation * (origin2 - origin1);
                Vector2 relativeDirection2 = dir1Rotation * direction2;
                float lambda = relativeOrigin2.y / relativeDirection2.y;
                Vector2 relativeResult = new Vector2(relativeOrigin2.x + (relativeDirection2.x * lambda), 0f);
                dir1Rotation = Quaternion.Inverse(dir1Rotation);
                return origin1 + (Vector2)((dir1Rotation) * relativeResult);
            }

            public static Path FindPath(Navigable_BHV startNav, int startFace, Vector3 startPoint, Navigable_BHV endNav, int endFace, Vector3 endPoint) {
                if ((startNav != null) && (endNav != null) && (startFace >= 0) && (endFace >= 0)) {
                    if (startNav == endNav) {
                        return new Path(startNav.FindPath(startFace, startPoint, endFace, endPoint));
                    }
                    else {
                        return new Path(AStarPathFinding(startNav, startFace, startPoint, endNav, endFace, endPoint));
                    }
                }
                return null;
            }
            public MeshPath FindPath(int startFace, Vector3 startPoint, int endFace, Vector3 endPoint) {
                Vector3 localStartPoint = transform.InverseTransformVector(startPoint);
                Vector3 localEndPoint = transform.InverseTransformVector(endPoint);
                List<Vertex> aStarResult = AStarPathFinding(startFace, localStartPoint, endFace, localEndPoint);
                if (aStarResult == null) {
                    return null;
                }
                return new MeshPath(this, startPoint, startFace, endPoint, endFace, aStarResult);
            }
            public MeshPath FindPath(Link startLink, int endFace, Vector3 endPoint) {
                Vector3 globalStartPoint = transform.TransformVector(startLink.GetLocalPoint(this));
                return FindPath(GetClosestPolyToPoint(globalStartPoint), globalStartPoint, endFace, endPoint);
            }
            public MeshPath FindPath(int startFace, Vector3 startPoint, Link endLink) {
                Vector3 globalEndPoint = transform.TransformVector(endLink.GetLocalPoint(this));
                return FindPath(startFace, startPoint, GetClosestPolyToPoint(globalEndPoint), globalEndPoint);
            }
            public MeshPath FindPath(Link startLink, Link endLink) {
                MeshPath cachedPath = pathCache.Find(p => p.startLink == startLink && p.endLink == endLink);
                if (cachedPath != null) {
                    return cachedPath;
                }
                cachedPath = pathCache.Find(p => p.endLink == startLink && p.startLink == endLink);
                if (cachedPath != null) {
                    return MeshPath.Reversed(cachedPath);
                }
                Vector3 globalStartPoint = transform.TransformVector(startLink.GetLocalPoint(this));
                Vector3 globalEndPoint = transform.TransformVector(endLink.GetLocalPoint(this));
                cachedPath = FindPath(GetClosestPolyToPoint(globalStartPoint), globalStartPoint, GetClosestPolyToPoint(globalEndPoint), globalEndPoint);
                pathCache.Add(cachedPath);
                return cachedPath;
            }
            public static List<MeshPath> AStarPathFinding(Navigable_BHV startNav, int startFace, Vector3 startPoint, Navigable_BHV endNav, int endFace, Vector3 endPoint) {
                List<Navigable_BHV> openNavList = new List<Navigable_BHV>();
                List<float> openNavTreeValue = new List<float>();
                List<Navigable_BHV> openNavTreeParent = new List<Navigable_BHV>();
                List<Link> openNavTreeParentLink = new List<Link>();
                List<MeshPath> openNavTreeParentMeshPath = new List<MeshPath>();
                List<bool> closedNavList = new List<bool>();

                /*
        foreach (Link l in startNav.linkList) {
            if ((l.originNav == startNav)||((l.finishNav == startNav)&&(!l.forwardOnly))) {//If link takes to other navigable
                MeshPath newMeshPath = startNav.FindPath(startFace, startPoint, l);//Inner pathfinding
                if (newMeshPath != null) {//If there is a path to link
                    openNavList.Add(l.GetOtherNav(startNav));
                    Path newPath = new Path(newMeshPath);
                    openNavTreeParentMeshPath.Add(newMeshPath);
                    openNavTreeValue.Add(newPath.completeCost);
                    openNavTreeParent.Add(startNav);
                    openNavTreeParentLink.Add(l);
                    closedNavList.Add(false);
                }
            }
        }

        int minIndex = -1;
        float minValue = float.MaxValue;
        for (int i = 0; i < openNavList.Count; i++) {
            float curValue = openNavTreeValue[i] 
                           + openNavTreeParentLink[i].estimateCost 
                           + Vector3.Distance(openNavTreeParentLink[i].GetGlobalPoint(openNavList[i]), endPoint);
            if (closedNavList[i] == false) {
                if (minValue > curValue) {
                    minValue = curValue;
                    minIndex = i;
                }
            }
        }
        if (minIndex < 0) {
            return null;
        }
        int curIndex = minIndex;
        Navigable_BHV curNav = openNavList[minIndex];
        closedNavList[minIndex] = true;


        while (curNav != endNav) {
            foreach (Link link in curNav.linkList) {
                Navigable_BHV navNeighbor = link.GetOtherNav(curNav);
                int index = openNavList.IndexOf(navNeighbor);
                float linkWeight = 0f;
                List<Poly> sharedOwners = curVertex.owners.FindAll(p => linkedVertex.owners.Contains(p));
                sharedOwners.ForEach(p => linkWeight += (weightList[polyList.IndexOf(p)]) / sharedOwners.Count);
                if ((index >= 0) || (openNavTreeParentLink[index] == link)){
                    if (closedNavList[index] == false) {
                        MeshPath newMeshPath = curNav.FindPath(openNavTreeParentLink[index], link);
                        if (newMeshPath != null) {//If there is a path to link
                            Path newPath = new Path(newMeshPath);
                            float newPathValue = openNavTreeValue[index] + openNavTreeParentLink[index].estimateCost + newPath.completeCost;
                            if (openVertexTreeValue[index] > newPathValue) {
                                openVertexTreeValue[index] = newPathValue;
                                openVertexTreeParent[index] = curVertex;
                            }
                        }
                    }
                }
                else {
                    MeshPath newMeshPath = curNav.FindPath(openNavTreeParentLink[index], link);//Inner pathfinding
                    if (newMeshPath != null) {//If there is a path to link
                        Path newPath = new Path(newMeshPath);
                        float newPathValue = openNavTreeValue[index] + openNavTreeParentLink[index].estimateCost + newPath.completeCost;
                        openNavList.Add(navNeighbor);
                        openNavTreeParentMeshPath.Add(newMeshPath);
                        openNavTreeValue.Add(newPathValue);
                        openNavTreeParent.Add(navNeighbor);
                        openNavTreeParentLink.Add(link);
                        closedNavList.Add(false);
                    }
                }
            }
            minIndex = -1;
            minValue = float.MaxValue;
            for (int i = 0; i < openVertexList.Count; i++) {
                float curValue = openVertexTreeValue[i] + Vector3.Distance(openVertexList[i].pos, endPoint);
                if (closedVertexList[i] == false) {
                    if (minValue > curValue) {
                        minValue = curValue;
                        minIndex = i;
                    }
                }
            }
                if (minIndex < 0) {
                return null;
            }
            curIndex = minIndex;
            curVertex = openVertexList[minIndex];
            closedVertexList[minIndex] = true;
        }
        
            */
                List<MeshPath> result = new List<MeshPath>();
                /*
                while (curVertex != null) {
                    result.Add(curVertex);
                    curVertex = openVertexTreeParent[openVertexList.IndexOf(curVertex)];
                }
                result.Reverse();
                */
                return result;
            }

            public void OnDrawGizmos() {
                Color c = Color.cyan;
                c.a = 0.5f;
                Gizmos.color = c;
                Gizmos.DrawMesh(mesh, transform.position, transform.rotation, transform.lossyScale);
                Gizmos.DrawWireMesh(mesh, transform.position, transform.rotation, transform.lossyScale);
                
                Color y = Color.yellow;
                y.a = 0.3f;
                Gizmos.color = y;
                Gizmos.DrawMesh(unfoldedMesh, transform.position, transform.rotation, transform.lossyScale);
                Gizmos.DrawWireMesh(unfoldedMesh, transform.position, transform.rotation, transform.lossyScale);
            }

        }
    }
}