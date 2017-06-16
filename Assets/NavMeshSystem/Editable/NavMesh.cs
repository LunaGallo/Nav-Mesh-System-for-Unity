using UnityEngine;
using System.Collections.Generic;

namespace NavMeshSystem {
    namespace Editable {
        public class NavMesh : MonoBehaviour {

            public List<Vertex> vertices = new List<Vertex>();
            public List<Edge> edges = new List<Edge>();
            public List<Face> faces = new List<Face>();

            public static NavMesh NewNavMesh() {
                GameObject obj = new GameObject();
                NavMesh navMesh = obj.AddComponent<NavMesh>();
                return navMesh;
            }
            public static NavMesh NewNavMesh(Mesh mesh) {
                NavMesh newMesh = NewNavMesh();
                foreach (Vector3 newPos in mesh.vertices) {
                    Vertex newVertex = Vertex.NewVertex(newMesh.transform);
                    newVertex.transform.localPosition = newPos;
                    newMesh.AddVertex(newVertex);
                }
                for (int i=0; i<mesh.triangles.Length; i+=3) {
                    Edge[] edges = new Edge[3];
                    for (int j=0; j<3; j++) {
                        int k = (j+1) % 3;
                        Vertex v1 = newMesh.vertices[mesh.triangles[i+j]];
                        Vertex v2 = newMesh.vertices[mesh.triangles[i+k]];
                        Edge newEdge = Edge.NewEdge(v1,v2,newMesh.transform);
                        Edge similar = newMesh.GetSimilarEdge(newEdge);
                        if (similar == null) {
                            edges[j] = newEdge;
                            newMesh.AddEdge(newEdge);
                        }
                        else {
                            edges[j] = similar;

                            newEdge.v1.edges.Remove(newEdge);
                            newEdge.v2.edges.Remove(newEdge);
                            DestroyImmediate(newEdge.gameObject);
                        }
                    }
                    Face newFace = Face.NewFace(newMesh.transform, edges);
                    newMesh.AddFace(newFace);
                }
                return newMesh;
            }

            public Mesh mesh {
                get {
                    Mesh newMesh = new Mesh();
                    List<int> vertexIdList = new List<int>();
                    foreach(Vertex vertex in vertices) {
                        vertexIdList.Add(newMesh.AddVertex(vertex.transform.localPosition, Vector3.up, Vector2.zero));
                    }
                    foreach (Face face in faces) {
                        List<int> faceVertices = new List<int>();
                        Vertex startingVertex = face.vertices[0];
                        faceVertices.Add(vertexIdList[vertices.IndexOf(startingVertex)]);
                        Edge currentEdge = face.edges.Find(e => e.ContainsVertex(startingVertex));
                        Vertex currentVertex = currentEdge.OtherVertex(startingVertex);
                        while (currentVertex != startingVertex) {
                            faceVertices.Add(vertexIdList[vertices.IndexOf(currentVertex)]);
                            currentEdge = face.edges.Find(e => ((e!=currentEdge)&&(e.ContainsVertex(currentVertex))));
                            currentVertex = currentEdge.OtherVertex(currentVertex);
                        }
                        newMesh.AddConvexPoly(faceVertices, 0, Vector3.up);
                    }
                    return newMesh;
                }
            }

            public void AddVertex(Vertex vertex) {
                if (!this.vertices.Contains(vertex))
                    this.vertices.Add(vertex);
            }
            public void RemoveVertex(Vertex vertex) {
                int count;
                // Removing Edges with this vertex
                count = edges.Count;
                for (int i = 0; i < count; i++)
                    if (edges[i].v1 == vertex || edges[i].v2 == vertex) {
                        GameObject.DestroyImmediate(this.edges[i].gameObject);
                        this.edges.RemoveAt(i--);
                        count--;
                    }
                // Removing Faces with this vertex
                count = faces.Count;
                for (int i = 0; i < count; i++)
                    if (faces[i].vertices.Contains(vertex)) {
                        GameObject.DestroyImmediate(this.faces[i].gameObject);
                        this.faces.RemoveAt(i--);
                        count--;
                    }
                // Removing vertex
                GameObject.DestroyImmediate(vertex.gameObject);
                this.vertices.Remove(vertex);
            }

            public void AddEdge(Edge edge) {
                // Already inserted
                if (this.edges.Contains(edge))
                    return;

                // Already similar edge
                if (this.HasSimilarEdge(edge)) {
                    edge.v1.edges.Remove(edge);
                    edge.v2.edges.Remove(edge);

                    GameObject.DestroyImmediate(edge.gameObject);
                    Debug.Log("Similar edge already exists! Aborting");
                    return;
                }

                

                this.edges.Add(edge);
            }
            public void RemoveEdge(Edge edge) {
                int count;
                // Removing Faces with this edge
                count = faces.Count;
                for (int i = 0; i < count; i++)
                    if (faces[i].edges.Contains(edge)) {
                        GameObject.DestroyImmediate(this.faces[i].gameObject);
                        this.faces.RemoveAt(i--);
                        count--;
                    }
                // Remove reference from all vertices
                edge.v1.edges.Remove(edge);
                edge.v2.edges.Remove(edge);
                // Removing edge
                GameObject.DestroyImmediate(edge.gameObject);
                this.edges.Remove(edge);
            }

            public void AddFace(Face face) {
                if (this.faces.Contains(face))
                    return;

                // Already similar face
                foreach (Face f in faces) {
                    bool same = (f.vertices.Count == face.vertices.Count);
                    if (!same)
                        continue;

                    foreach (Vertex v in f.vertices)
                        if (!face.vertices.Contains(v)) {
                            same = false;
                            break;
                        }

                    if (same) {
                        // Remove reference from all vertices
                        foreach (Vertex v in face.vertices)
                            v.faces.Remove(face);
                        // Remove reference from all Edges
                        foreach (Edge e in face.edges)
                            e.faces.Remove(face);

                        GameObject.DestroyImmediate(face.gameObject);
                        Debug.Log("Similar face already exists! Aborting");
                        return;
                    }
                }


                this.faces.Add(face);
            }
            public void RemoveFace(Face face) {
                // Remove reference from all vertices
                foreach (Vertex v in face.vertices)
                    v.faces.Remove(face);
                // Remove reference from all edges
                foreach (Edge e in face.edges)
                    e.faces.Remove(face);
                // removing face
                GameObject.DestroyImmediate(face.gameObject);
                this.faces.Remove(face);
            }

            public static Edge[] IsCircular(Vertex[] v) {
                if (v.Length < 3)
                    return null;

                List<Edge> ret = new List<Edge>();
                List<Vertex> passed = new List<Vertex>();
                Edge[] usefull = new Edge[2];

                Vertex cur = v[0];
                passed.Add(cur);


                Vertex nextV;
                Edge nextE;


                while (v.Length != ret.Count) {
                    // If last one reuse the first
                    if (v.Length == ret.Count + 1)
                        passed.RemoveAt(0);

                    // Check vertex only has edges to two of the selected
                    int edgeCount = 0;
                    foreach (Edge e in cur.edges)
                        foreach (Vertex x in v)
                            if (x != cur && (e.v1 == x || e.v2 == x)) {
                                // To manny possible ways... abort
                                if (edgeCount > 1)
                                    return null;
                                usefull[edgeCount++] = e;
                                break;
                            }

                    // Select the edge to a vertex not used yet
                    nextV = null;
                    nextE = null;
                    if (!passed.Contains(usefull[0].v1)) {
                        nextV = usefull[0].v1;
                        nextE = usefull[0];
                    }
                    else if (!passed.Contains(usefull[0].v2)) {
                        nextV = usefull[0].v2;
                        nextE = usefull[0];
                    }
                    else if (!passed.Contains(usefull[1].v1)) {
                        nextV = usefull[1].v1;
                        nextE = usefull[1];
                    }
                    else if (!passed.Contains(usefull[1].v2)) {
                        nextV = usefull[1].v2;
                        nextE = usefull[1];
                    }

                    // If did not find next
                    if (nextV == null)
                        return null;

                    // Add the edge to solution
                    ret.Add(nextE);
                    // Set next vertex
                    cur = nextV;
                    passed.Add(cur);
                }

                return ret.ToArray();
            }
            public bool HasSimilarEdge(Edge edge) {
                foreach (Edge e in edges) {
                    if ((e.v1 == edge.v1 && e.v2 == edge.v2) ||
                        (e.v1 == edge.v2 && e.v2 == edge.v1)) {
                        return true;
                    }
                }
                return false;
            }
            public Edge GetSimilarEdge(Edge edge) {
                foreach (Edge e in edges) {
                    if ((e.v1 == edge.v1 && e.v2 == edge.v2) ||
                        (e.v1 == edge.v2 && e.v2 == edge.v1)) {
                        return e;
                    }
                }
                return null;
            }

        }
    }
}