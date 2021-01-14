using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

public class Pathfinding : MonoBehaviour {

    Grid grid;

    void Awake () {
        grid = GetComponent<Grid> ();
    }

    public void FindPath (PathRequest request, Action<PathResult> callback) {
        Stopwatch sw = new Stopwatch ();
        sw.Start ();

        Vector3[] waypoints = new Vector3[0];
        bool pathSuccess = false;

        Node startNode = grid.NodeFromWorldPoint (request.pathStart);
        Node targetNode = grid.NodeFromWorldPoint (request.pathEnd);

        if (startNode.isWalkable && targetNode.isWalkable) {

            // List<Node> openSet = new List<Node> ();
            Heap<Node> openSet = new Heap<Node> (grid.MaxSize);
            HashSet<Node> closeSet = new HashSet<Node> ();

            openSet.Add (startNode);

            while (openSet.Count > 0) {

                /* Open Set implemented with List (expensive calculation cost) */
                // Node currentNode = openSet[0];
                // for (int i = 1; i < openSet.Count; i++) {
                //     if (openSet[i].fCost < currentNode.fCost || openSet[i].fCost == currentNode.fCost && openSet[i].hCost < currentNode.hCost) {
                //         currentNode = openSet[i];
                //     }
                // }
                // openSet.Remove (currentNode);

                /* Open Set implemented with Heap */
                Node currentNode = openSet.RemoveFirst ();
                closeSet.Add (currentNode);

                if (currentNode == targetNode) {
                    sw.Stop ();
                    print ("Path found: " + sw.ElapsedMilliseconds + " ms");
                    pathSuccess = true;
                    break;
                }

                foreach (Node neighbor in grid.GetNeighbors (currentNode)) {
                    if (!neighbor.isWalkable || closeSet.Contains (neighbor)) {
                        continue;
                    }

                    int newMovementCostToNeighbor = currentNode.gCost + GetDistance (currentNode, neighbor) + neighbor.movementPenalty;
                    if (newMovementCostToNeighbor < neighbor.gCost || !openSet.Contains (neighbor)) {
                        neighbor.gCost = newMovementCostToNeighbor;
                        neighbor.hCost = GetDistance (neighbor, targetNode);
                        neighbor.parent = currentNode;

                        if (!openSet.Contains (neighbor)) {
                            openSet.Add (neighbor);
                        } else {
                            openSet.UpdateItem (neighbor);
                        }
                    }
                }
            }
        }

        if (pathSuccess) {
            waypoints = RetracePath (startNode, targetNode);
            pathSuccess = waypoints.Length > 0;
        }

        callback (new PathResult (waypoints, pathSuccess, request.callback));
    }

    Vector3[] RetracePath (Node startNode, Node endNode) {
        List<Node> path = new List<Node> ();
        Node currentNode = endNode;
        while (currentNode != startNode) {
            path.Add (currentNode);
            currentNode = currentNode.parent;
        }
        Vector3[] waypoints = SimplifyPath (path);
        Array.Reverse (waypoints);
        return waypoints;
    }

    Vector3[] SimplifyPath (List<Node> path) {
        List<Vector3> waypoints = new List<Vector3> ();
        Vector2 directionOld = Vector2.zero;
        for (int i = 1; i < path.Count; i++) {
            Vector2 directionNew = new Vector2 (path[i - 1].gridX - path[i].gridX, path[i - 1].gridY - path[i].gridY);
            if (directionNew != directionOld) {
                waypoints.Add (path[i].worldPosition);
            }
            directionOld = directionNew;
        }
        return waypoints.ToArray ();
    }

    int GetDistance (Node nodeA, Node nodeB) {
        int distX = Mathf.Abs (nodeA.gridX - nodeB.gridX);
        int distY = Mathf.Abs (nodeA.gridY - nodeB.gridY);

        return distX > distY ? 14 * distY + 10 * (distX - distY) : 14 * distX + 10 * (distY - distX);
    }
}