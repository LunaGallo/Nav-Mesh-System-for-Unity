using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;


namespace NavMeshSystem {
    namespace Editable {
        public class Vertex : MonoBehaviour {
            public List<Face> faces = new List<Face>();
            public List<Edge> edges = new List<Edge>();

            private static int count = 0;


            public static Vertex NewVertex(Transform parent) {
                GameObject obj = new GameObject();
                Vertex vertex = obj.AddComponent<Vertex>();
                vertex.name = "vertex " + count++;
                obj.transform.parent = parent;

                return vertex;
            }


            void OnDrawGizmos() {
                Draw(Color.green);
            }
            void OnDrawGizmosSelected() {
                Draw(Color.blue);
            }

            public void Draw(Color color) {
                float size = 1f;
                Gizmos.color = color;
                Gizmos.DrawSphere(transform.position, size);
            }
        }
    }
}