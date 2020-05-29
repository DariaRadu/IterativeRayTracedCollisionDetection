using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class RayCasting : MonoBehaviour
{
    [SerializeField]
    private float maxRayDistance = 1;
    [SerializeField]
    private bool showWireframe = true;
    [SerializeField]
    private GameObject objToCollide;
    [SerializeField]
    private float maxIterations = 100;
    [SerializeField]
    private float displacementThreshold = 1;

    private Mesh mesh, meshToCollide;
    private Vector3[] vertices, normals;
    private Vector3[] verticesToCollide;
    private int[] trianglesToCollide;
    private int[][] prevTriangles;

    private float maxDisplacement;
    private Vector3 t, tni;
    private Quaternion q, qni;

    private Vector3 targetPosition;

    private void Start()
    {
        mesh = gameObject.GetComponent<MeshFilter>().sharedMesh;
        vertices = mesh.vertices;
        normals = mesh.normals;

        meshToCollide = objToCollide.GetComponent<MeshFilter>().sharedMesh;
        verticesToCollide = meshToCollide.vertices;
        trianglesToCollide = meshToCollide.triangles;

        prevTriangles = new int[vertices.Length][];

        t = this.transform.InverseTransformPoint(objToCollide.transform.position);
        q = Quaternion.FromToRotation(this.transform.position, objToCollide.transform.position);
        
        // Initialization to avoid errors in first time step
        tni = t;
        qni = q;

        // For animation
        targetPosition = new Vector3(transform.position.x - 3, transform.position.y, transform.position.z);
    }

    private void Update()
    {
        // Small Animation
        float step = 0.5f * Time.deltaTime;
        transform.position = Vector3.MoveTowards(transform.position, targetPosition, step);
    }
    private void FixedUpdate()
    {
        CastRaysFromVertices(vertices, normals);
    }

    private void OnDrawGizmos()
    {
        if (showWireframe)
        {
            Gizmos.color = Color.green; // wireframe color
            Gizmos.DrawWireMesh(mesh, this.transform.position, this.transform.rotation);
        }
    }

    void CastRaysFromVertices(Vector3[] vertices, Vector3[] normals)
    {

        for (int i = 0; i < vertices.Length; i++)
        {
            // Cast the ray and visualize it
            Ray ray = new Ray(transform.position + vertices[i], -normals[i]);
            Debug.DrawRay(ray.origin, -normals[i] * maxRayDistance, Color.yellow);
            
            // Set the current (t, q)
            t = this.transform.InverseTransformPoint(objToCollide.transform.position);
            q = Quaternion.FromToRotation(this.transform.position, objToCollide.transform.position);

            maxDisplacement = CalculateMaxDisplacement();

            if (prevTriangles[i] == null || maxDisplacement > displacementThreshold)
            {
                prevTriangles[i] = null;
                RayTracingCollisionDetection(ray, i);
            }
            else
            {
                int[] triangle = prevTriangles[i];
                if (IterativeRayTriangleIntersection(ray, ref triangle))
                {
                    prevTriangles[i] = triangle;
                }
            }
        }
    }

    void RayTracingCollisionDetection(Ray ray, int j)
    {
        Vector3 hitPoint = Vector3.zero;
        Vector3 hitNormal = Vector3.forward;

        for (int i = 0; i < trianglesToCollide.Length - 3; i += 3)
        {
            Vector3 p1 = objToCollide.transform.position + verticesToCollide[trianglesToCollide[i]];
            Vector3 p2 = objToCollide.transform.position + verticesToCollide[trianglesToCollide[i + 1]];
            Vector3 p3 = objToCollide.transform.position + verticesToCollide[trianglesToCollide[i + 2]];

            float u = 0, v = 0;
            if (AiG.IntersectionTests.RayTriangleIntersection(ray, p1, p2, p3, ref hitPoint, ref hitNormal, ref u, ref v))
            {
                // Check if the ray has not left the object yet (for the primitive unscaled objects, it's 1)
                if (Vector3.Distance(ray.origin, hitPoint) > 1) continue;

                Debug.DrawLine(ray.origin, hitPoint, Color.magenta, 500);
                //Debug.DrawLine(hitPoint, hitNormal, Color.blue, 500);

                // Set the (t,q) at last non-iterative step
                tni = t = this.transform.InverseTransformPoint(objToCollide.transform.position);
                qni = Quaternion.FromToRotation(this.transform.position, objToCollide.transform.position);
                int[] triangle = { trianglesToCollide[i], trianglesToCollide[i + 1], trianglesToCollide[i + 2] };
                prevTriangles[j] = triangle;
            }

        }
    }

    private bool IterativeRayTriangleIntersection(Ray ray, ref int[] triangle)
    {
        Vector3 hitPoint = Vector3.zero;
        Vector3 hitNormal = Vector3.forward;
        float u = 0, v = 0;
        for (int i = 0; i < maxIterations; i++)
        {

            Vector3 p1 = objToCollide.transform.position + verticesToCollide[triangle[0]];
            Vector3 p2 = objToCollide.transform.position + verticesToCollide[triangle[1]];
            Vector3 p3 = objToCollide.transform.position + verticesToCollide[triangle[2]];

            if (AiG.IntersectionTests.RayTriangleIntersection(ray, p1, p2, p3, ref hitPoint, ref hitNormal, ref u, ref v))
            {
                // Check if the ray has not left the object yet (for the primitive unscaled objects, it's 1)
                if (Vector3.Distance(ray.origin, hitPoint) > 1) continue;

                Debug.DrawLine(ray.origin, hitPoint, Color.red, 3);
                //Debug.DrawLine(hitPoint, hitNormal, Color.blue, 500);
                return true;
            } else
            {
                Vector3[] edge = FindClosestEdge(p1, p2, p3, u, v);
                edge[0] = edge[0] - objToCollide.transform.position;
                edge[1] = edge[1] - objToCollide.transform.position;
                
                triangle = FindAdjacentTriangle(edge, triangle);
            }
        }

        return false;
    }

    private Vector3[] FindClosestEdge(Vector3 p1, Vector3 p2, Vector3 p3, float u, float v)
    {
        // if u < 0, RAY CLOSEST TO EDGE 1
        if (u < 0) return new Vector3[] { p1, p2 };

        // if v < 0 RAY CLOSEST TO EDGE 2
        if (v < 0) return new Vector3[] { p1, p3 };

        // u + v > 1 RAY CLOSEST TO EDGE 3
        if (u + v > 1) return new Vector3[] { p2, p3 };

        return new Vector3[] { Vector3.zero, Vector3.zero };
    }

    private int[] FindAdjacentTriangle(Vector3[] edge, int[] triangle)
    {
        for (int i = 1; i < trianglesToCollide.Length - 1; i++)
        {
            Vector3 v = verticesToCollide[trianglesToCollide[i]];
            // Check if v is one of the points of the edges
            if (v == edge[0] || v == edge[1])
            {
                int triNum = i / 3;
                // Check position of the vertex index in the triangle list
                // Needed in order to check if the found triangle is not the same
                if (i % 3 == 0)
                {
                    Vector3 v2 = verticesToCollide[trianglesToCollide[i + 1]];
                    if (v2 == edge[0] || v2 == edge[1])
                    {
                        int[] newTriangle = new int[] { trianglesToCollide[triNum],
                                                    trianglesToCollide[triNum + 1], trianglesToCollide[triNum +2]};
                        if(newTriangle[2] != triangle[2]) return newTriangle;
                    }
                } else if (i % 3 == 1)
                {
                    Vector3 prevV = verticesToCollide[trianglesToCollide[i - 1]];
                    Vector3 nextV = verticesToCollide[trianglesToCollide[i + 1]];
                    if (prevV == edge[0] || prevV == edge[1] || nextV == edge[0] || nextV == edge[1])
                    {
                        int[] newTriangle = new int[] { trianglesToCollide[triNum],
                                                    trianglesToCollide[triNum + 1], trianglesToCollide[triNum +2]};
                        if (triangle[0] != newTriangle[0] && triangle[2] != newTriangle[2]) return newTriangle;
                    }
                } else if (i % 3 == 2)
                {
                    Vector3 v2 = verticesToCollide[trianglesToCollide[i - 1]];
                    if (v2 == edge[0] || v2 == edge[1])
                    {
                        int[] newTriangle = new int[] { trianglesToCollide[triNum],
                                                    trianglesToCollide[triNum + 1], trianglesToCollide[triNum +2]};
                        if (newTriangle[0] != triangle[0]) return newTriangle;
                    }
                }
            }
        }
        return triangle;
    }

    private float CalculateMaxDisplacement()
    {
        float distance = Vector3.Magnitude(tni - t);
        float angle = Quaternion.Angle(qni, q);
        float r1 = this.GetComponent<SphereCollider>().radius;
        float r2 = objToCollide.GetComponent<SphereCollider>().radius;
        float maxRadius = Mathf.Max(r1, r2);

        float maxDisplacement = distance + angle * maxRadius;

        return maxDisplacement;
    }

}
