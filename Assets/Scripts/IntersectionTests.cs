using UnityEngine;

namespace AiG
{
    // Intersection Test Base taken from class (Made by Henrique Galvan Debarba)
    public static class IntersectionTests
    {
        public static bool RayTriangleIntersection(Ray ray, Vector3 p1, Vector3 p2, Vector3 p3,
                                                ref Vector3 hitPoint, ref Vector3 hitNormal, 
                                                ref float u, ref float v)
        {
            Vector3 e1 = p2 - p1;
            Vector3 e2 = p3 - p1;
            Vector3 q = Vector3.Cross(ray.direction, e2);
            float a = Vector3.Dot(e1, q);

            // for numerical stability, a = 0 means that triangle plane and ray are parallel
            if (Mathf.Abs(a) < 10e-5) return false;

            float f = 1.0f / a;
            Vector3 s = ray.origin - p1;
            u = f * Vector3.Dot(s, q);

            // if u < 0, intersection with plane is not within the triangle
            if (u < 0) return false;

            Vector3 r = Vector3.Cross(s, e1);
            v = f * Vector3.Dot(ray.direction, r);

            // if v < 0 or u + v > 1, intersection with plane is not within the triangle
            if (v < 0 || u + v > 1) return false;

            float t = f * Vector3.Dot(e2, r);
            hitPoint = ray.origin + ray.direction * t;
            hitNormal = Vector3.Cross(e1, e2).normalized;

            // t >= 0 RAY INTERSECTS TRIANGLE, COLLISION DETECTED
            // else ray behind triangle, discard
            if (t >= 0) return true;
            return false;
        }
    }
}
