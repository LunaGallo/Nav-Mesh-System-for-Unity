using UnityEngine;
using System.Collections;
using System.Collections.Generic;


namespace NavMeshSystem {
    namespace Editable {
        public class Edge : MonoBehaviour {
            public List<Face> faces = new List<Face>();
            public Vertex v1, v2;

            private static int count = 0;

            public static Edge NewEdge(Vertex v1, Vertex v2, Transform parent) {
                GameObject obj = new GameObject();
                Edge edge = obj.AddComponent<Edge>();
                edge.v1 = v1;
                edge.v2 = v2;
                edge.name = "edge " + count++;
                obj.transform.parent = parent;
                obj.transform.position = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);

                v1.edges.Add(edge);
                v2.edges.Add(edge);

                return edge;
            }

            public bool ContainsVertex(Vertex vertex) {
                return ((v1 == vertex) || (v2 == vertex));
            }
            public Vertex OtherVertex(Vertex vertex) {
                if (v1 == vertex) {
                    return v2;
                }
                else if (v2 == vertex) {
                    return v1;
                }
                else {
                    return null;
                }
            }

            void OnDrawGizmos() {
                Draw(Color.green);
            }
            void OnDrawGizmosSelected() {
                Draw(Color.blue);
            }

            public void Draw(Color color) {
                Gizmos.color = color;

                Vector3 pos = (v1.transform.position + v2.transform.position) / 2;
                Gizmos.DrawCube(pos, Vector3.one);

                Gizmos.DrawLine(this.v1.transform.position, this.v2.transform.position);
            }
        }
    }
}