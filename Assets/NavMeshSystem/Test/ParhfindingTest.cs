using System.Collections.Generic;
using UnityEngine;
using NavMeshSystem.Core;

public class ParhfindingTest : MonoBehaviour {

    public NavPosition_BHV startPosition;
    public NavPosition_BHV endPosition;
    public Navigable_BHV navigableRef;

    public Navigable_BHV.MeshPath path;

    private void Update() {
        path = navigableRef.FindPath(startPosition.currentRegionIndex, startPosition.realPosition,
                              endPosition.currentRegionIndex, endPosition.realPosition);
        DrawPath();
    }

    public void DrawPath() {
        if (path != null) {
            path.allPoints.ForEachConsecutivePair((f,s) => Debug.DrawLine(f,s,Color.red));
        }
    }

    public void OnDrawGizmos() {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(startPosition.realPosition, 1f);
        Vector3 startPoint = startPosition.realPosition;
        Vector3 endPoint = endPosition.realPosition + new Vector3(10f, -10f, 0f);
        float firstEdgeParameter = (10f - startPoint.x) / (endPoint.x - startPoint.x);
        Vector3 firstEdgePosition = new Vector3(10f, 0f, startPoint.z + (firstEdgeParameter * (endPoint.z - startPoint.z)));
        float secondEdgeParameter = (20f - startPoint.x) / (endPoint.x - startPoint.x);
        Vector3 secondEdgePosition = new Vector3(10f, 10f, startPoint.z + (secondEdgeParameter * (endPoint.z - startPoint.z)));
        Gizmos.DrawLine(startPosition.realPosition, firstEdgePosition);
        Gizmos.DrawLine(firstEdgePosition, secondEdgePosition);
        Gizmos.DrawLine(secondEdgePosition, endPosition.realPosition);
        Gizmos.DrawSphere(endPosition.realPosition, 1f);
    }

}
