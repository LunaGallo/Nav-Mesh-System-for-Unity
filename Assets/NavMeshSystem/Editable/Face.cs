using UnityEngine;
using System.Collections;
using System.Collections.Generic;


namespace NavMeshSystem {
    namespace Editable {
        public class Face : MonoBehaviour {

            public List<Vertex> vertices = new List<Vertex>();
            public List<Edge> edges = new List<Edge>();

            private static int count = 0;
            


            public static Face NewFace(Transform parent, Edge[] edges) {
                GameObject obj = new GameObject();
                Face face = obj.AddComponent<Face>();
                face.name = "face " + count++;
                obj.transform.parent = parent;
                obj.transform.position = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);

                face.edges.AddRange(edges);
                // Add vertices
                for (int i = 0; i < edges.Length; i++) {
                    if (!face.vertices.Contains(edges[i].v1)) {
                        face.vertices.Add(edges[i].v1);
                    }
                    if (!face.vertices.Contains(edges[i].v2)) {
                        face.vertices.Add(edges[i].v2);
                    }
                }

                foreach (Vertex v in face.vertices)
                    v.faces.Add(face);
                foreach (Edge e in face.edges)
                    e.faces.Add(face);

                return face;
            }


            void OnDrawGizmos() {
                Draw(Color.yellow);
            }
            void OnDrawGizmosSelected() {
                Draw(Color.cyan);
            }

            public void Draw(Color color) {
                float size = 1f;

                // Calc position to draw
                Vector3 pos = Vector3.zero;
                foreach (Vertex v in this.vertices)
                    pos += v.transform.position;
                pos /= this.vertices.Count;

                // Draw
                Gizmos.color = color;
                Gizmos.DrawCube(pos, Vector3.one * size);

                // Draw connections
                foreach (Vertex v in vertices) {
                    Gizmos.DrawLine(pos, v.transform.position);
                }
                
            }
        }
    }
}