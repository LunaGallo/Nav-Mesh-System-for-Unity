using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif


using NavMeshSystem.Core;

namespace NavMeshSystem {
    namespace Editable {
            public class NavMeshEditorWindow : EditorWindow {
            public static float dotSize = 1f;
            private NavMesh editMesh;
            
            public Navigable_BHV navigable;


            // Add menu item named "NavMeshEditor" to the Window menu
            [MenuItem("Window/NavMeshEditor")]
            public static void ShowWindow() {
                EditorWindow.GetWindow(typeof(NavMeshEditorWindow));
            }


            // The window GUI
            void OnGUI() {

                // Start/Stop button
                if (this.editMesh == null) {
                    if (navigable != null && GUILayout.Button("Start")) {
                        this.StartEdit();
                    }
                }
                else {
                    if (GUILayout.Button("Stop")) {
                        this.StopEdit();
                    }
                }

                // If started
                if (this.editMesh != null) {
                    // Buttons
                    GUILayout.BeginHorizontal();
                    {
                        // Labels
                        GUILayout.BeginVertical();
                        {
                            GUILayout.Label("Vertex: ");
                            GUILayout.Label("Edge: ");
                            GUILayout.Label("Face: ");
                        }
                        GUILayout.EndVertical();
                        // New
                        GUILayout.BeginVertical();
                        {
                            if (GUILayout.Button("New"))
                                NewVertex();
                            if (GUILayout.Button("New"))
                                NewEdges();
                            if (GUILayout.Button("New"))
                                NewFace();
                        }
                        GUILayout.EndVertical();
                        // Delete
                        GUILayout.BeginVertical();
                        {
                            if (GUILayout.Button("Delete"))
                                RemoveVertexes();
                            if (GUILayout.Button("Delete"))
                                RemoveEdges();
                            if (GUILayout.Button("Delete"))
                                RemoveFaces();
                        }
                        GUILayout.EndVertical();
                    }
                    GUILayout.EndHorizontal();

                    // Save
                    if (GUILayout.Button("Save mesh to File")) {
                        Save();
                    }
                    // Apply
                    if (GUILayout.Button("Apply Changes to Navigable")) {
                        Apply();
                    }
                }

                navigable = (Navigable_BHV)EditorGUILayout.ObjectField("Nav Mesh: ", navigable, typeof(Navigable_BHV), true);

                if (this.editMesh != null) {
                    Object[] selected = GetSelectedWithComponet(typeof(Face));
                    if (selected.Length > 0) {
                        List<Face> selectedFaces = new List<Face>();
                        for (int i = 0; i < selected.Length; i++) {
                            selectedFaces.Add(selected[i] as Face);
                        }
                        float weight = navigable.weightList[editMesh.faces.IndexOf(selectedFaces[0])];
                        if (!selectedFaces.TrueForAll(f => navigable.weightList[editMesh.faces.IndexOf(f)] == weight)) {
                            weight = float.NaN;
                        }
                        weight = EditorGUILayout.FloatField("Region weight", weight);
                        if (!float.IsNaN(weight)) {
                            weight = Mathf.Max(1f, weight);
                            selectedFaces.ForEach(f => navigable.weightList[editMesh.faces.IndexOf(f)] = weight);
                        }
                    }
                }
            }

            void OnDestroy() {
                this.StopEdit();
            }

            
            #region Button Functions

            #region Faces
            void NewFace() {
                // Get selected vertices
                Object[] selected = GetSelectedWithComponet(typeof(Vertex));
                Vertex[] sel = new Vertex[selected.Length];
                for (int i = 0; i < sel.Length; i++) {
                    sel[i] = selected[i] as Vertex;
                }

                // Create face
                Edge[] circle = NavMesh.IsCircular(sel);
                if (circle != null) {
                    this.editMesh.AddFace(Face.NewFace(this.editMesh.transform, circle));
                }
                else {
                    Debug.Log("Selected vertices do not posess edges (or posess more than one way) that form a face!");
                }
            }

            void RemoveFaces() {
                // Get selected faces
                Object[] selected = GetSelectedWithComponet(typeof(Face));

                // Removing
                for (int i = 0; i < selected.Length; i++)
                    this.editMesh.RemoveFace(selected[i] as Face);
            }
            #endregion

            #region Vertices
            void NewVertex() {
                // Create Vertex
                Vertex newVertex = Vertex.NewVertex(this.editMesh.transform);
                this.editMesh.AddVertex(newVertex);

                // Get selected vertices
                Object[] selected = GetSelectedWithComponet(typeof(Vertex));

                // Adding edges
                Vector3 midPoint = Vector3.zero;
                if (selected.Length > 0) {
                    for (int i = 0; i < selected.Length; i++) {
                        Vertex selVertex = selected[i] as Vertex;
                        this.editMesh.AddEdge(Edge.NewEdge(selVertex, newVertex, this.editMesh.transform));
                        midPoint += (selVertex.transform.position);
                    }
                    midPoint /= selected.Length;
                }
                newVertex.transform.position = midPoint;

                Object[] newSelection = new Object[1];
                newSelection[0] = newVertex.gameObject;
                Selection.objects = newSelection;

            }

            void RemoveVertexes() {
                // Get selected vertices
                Object[] selected = GetSelectedWithComponet(typeof(Vertex));

                // Removing
                for (int i = 0; i < selected.Length; i++)
                    this.editMesh.RemoveVertex(selected[i] as Vertex);
            }
            #endregion

            #region Edges
            void NewEdges() {
                // Get selected vertices
                Object[] selected = GetSelectedWithComponet(typeof(Vertex));

                // Adding edges
                for (int i = 0; i < selected.Length - 1; i++)
                    for (int j = i + 1; j < selected.Length; j++)
                        this.editMesh.AddEdge(Edge.NewEdge(selected[i] as Vertex, selected[j] as Vertex, this.editMesh.transform));
            }

            void RemoveEdges() {
                // Get selected Edges
                Object[] selected = GetSelectedWithComponet(typeof(Edge));

                // Removing
                for (int i = 0; i < selected.Length; i++)
                    this.editMesh.RemoveEdge(selected[i] as Edge);
            }
            #endregion

            #region Start/Stop
            void StartEdit() {
                Load();
                editMesh.name = "Editable " + navigable.name;
            }
            
            void StopEdit() {
                if (editMesh != null) {
                    int option = EditorUtility.DisplayDialogComplex("Stopping navmesh edit", "Do you wish to save it to file?", "Save mesh", "Don't save", "Cancel");
                    switch (option) {
                        case 0: // Save mesh
                            Save();
                            DestroyImmediate(editMesh.gameObject);
                            editMesh = null;
                            break;
                        case 1: // Don't save.
                            DestroyImmediate(editMesh.gameObject);
                            editMesh = null;
                            break;
                    }
                }
            }
            #endregion

            #region Save/Load/Apply
            void Save() {
                string fileName = EditorUtility.SaveFilePanel("Save Mesh Asset", "Assets/", navigable.name, "asset");
                AssetDatabase.CreateAsset(editMesh.mesh, FileUtil.GetProjectRelativePath(fileName));
                AssetDatabase.SaveAssets();
            }

            void Load() {
                if (navigable != null) {
                    if (navigable.mesh == null) {
                        editMesh = NavMesh.NewNavMesh();
                    }
                    else {
                        editMesh = NavMesh.NewNavMesh(navigable.mesh);
                    }
                    editMesh.transform.SetParent(navigable.transform);
                    editMesh.transform.localPosition = Vector3.zero;
                }
            }

            void Apply() {
                Undo.RecordObject(navigable, "Apply Changes to Navigable");
                navigable.mesh = editMesh.mesh;
            }
            #endregion

            #endregion


            Object[] GetSelectedWithComponet(System.Type component) {
                // Get selected vertices
                Transform[] selected = Selection.transforms;
                System.Collections.Generic.List<Object> selectedWithComponent = new System.Collections.Generic.List<Object>();

                foreach (Transform tr in selected) {
                    Object o = tr.GetComponent(component);
                    if (o != null)
                        selectedWithComponent.Add(o);
                }

                return selectedWithComponent.ToArray();
            }

        }
    }
}
