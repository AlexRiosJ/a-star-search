﻿using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

public class Pathfinding : MonoBehaviour {

    public Transform seeker, target;

    Grid grid;

    void Awake () {
        grid = GetComponent<Grid> ();
    }

    void Update () {
        if (Input.GetButtonDown ("Jump")) {
            FindPath (seeker.position, target.position);
        }
    }

    void FindPath (Vector3 startPosition, Vector3 targetPosition) {
        Stopwatch sw = new Stopwatch ();
        sw.Start ();
        Node startNode = grid.NodeFromWorldPoint (startPosition);
        Node targetNode = grid.NodeFromWorldPoint (targetPosition);

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
                RetracePath (startNode, targetNode);
                break;
            }

            foreach (Node neighbor in grid.GetNeighbors (currentNode)) {
                if (!neighbor.isWalkable || closeSet.Contains (neighbor)) {
                    continue;
                }

                int newMovementCostToNeighbor = currentNode.gCost + GetDistance (currentNode, neighbor);
                if (newMovementCostToNeighbor < neighbor.gCost || !openSet.Contains (neighbor)) {
                    neighbor.gCost = newMovementCostToNeighbor;
                    neighbor.hCost = GetDistance (neighbor, targetNode);
                    neighbor.parent = currentNode;

                    if (!openSet.Contains (neighbor)) {
                        openSet.Add (neighbor);
                    }
                }
            }
        }
    }

    void RetracePath (Node startNode, Node endNode) {
        List<Node> path = new List<Node> ();
        Node currentNode = endNode;
        while (currentNode != startNode) {
            path.Add (currentNode);
            currentNode = currentNode.parent;
        }

        path.Reverse ();
        grid.path = path;
    }

    int GetDistance (Node nodeA, Node nodeB) {
        int distX = Mathf.Abs (nodeA.gridX - nodeB.gridX);
        int distY = Mathf.Abs (nodeA.gridY - nodeB.gridY);

        return distX > distY ? 14 * distY + 10 * (distX - distY) : 14 * distX + 10 * (distY - distX);
    }
}