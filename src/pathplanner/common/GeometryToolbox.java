package pathplanner.common;


import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class GeometryToolbox
{

    /**
     * This is a java program to find a points in convex hull using quick hull method source: Alexander Hrishov's website URL: http://www.ahristov.com/tutorial/geometry-games/convex-hull.html
     * 
     * @param points
     * @return
     */
    public static List<Vector2D> quickHull(List<Vector2D> points)
    {
        ArrayList<Vector2D> convexHull = new ArrayList<Vector2D>();
        if (points.size() < 3) return new ArrayList<Vector2D>(points);

        int minPoint = -1, maxPoint = -1;
        double minX = Integer.MAX_VALUE;
        double maxX = Integer.MIN_VALUE;
        for (int i = 0; i < points.size(); i++)
        {
            if (points.get(i).x < minX)
            {
                minX = points.get(i).x;
                minPoint = i;
            }
            if (points.get(i).x > maxX)
            {
                maxX = points.get(i).x;
                maxPoint = i;
            }
        }
        Vector2D A = points.get(minPoint);
        Vector2D B = points.get(maxPoint);
        convexHull.add(A);
        convexHull.add(B);
        points.remove(A);
        points.remove(B);

        ArrayList<Vector2D> leftSet = new ArrayList<Vector2D>();
        ArrayList<Vector2D> rightSet = new ArrayList<Vector2D>();

        for (int i = 0; i < points.size(); i++)
        {
            Vector2D p = points.get(i);
            if (pointLocation(A, B, p) == -1) leftSet.add(p);
            else if (pointLocation(A, B, p) == 1) rightSet.add(p);
        }
        hullSet(A, B, rightSet, convexHull);
        hullSet(B, A, leftSet, convexHull);

        Collections.reverse(convexHull); // Make points CCW
        return convexHull;
    }

    /**
     * Also part of the QuickHull algorithm
     * 
     * @param A
     * @param B
     * @param C
     * @return
     */
    private static double distance(Vector2D A, Vector2D B, Vector2D C)
    {
        double ABx = B.x - A.x;
        double ABy = B.y - A.y;
        double num = ABx * (A.y - C.y) - ABy * (A.x - C.x);
        if (num < 0) num = -num;
        return num;
    }

    /**
     * Also part of the QuickHull algorithm
     * 
     * @param A
     * @param B
     * @param set
     * @param hull
     */
    private static void hullSet(Vector2D A, Vector2D B,
            ArrayList<Vector2D> set,
            ArrayList<Vector2D> hull)
    {
        int insertPosition = hull.indexOf(B);
        if (set.size() == 0) return;
        if (set.size() == 1)
        {
            Vector2D p = set.get(0);
            set.remove(p);
            hull.add(insertPosition, p);
            return;
        }
        double dist = Integer.MIN_VALUE;
        int furthestPoint = -1;
        for (int i = 0; i < set.size(); i++)
        {
            Vector2D p = set.get(i);
            double distance = distance(A, B, p);
            if (distance > dist)
            {
                dist = distance;
                furthestPoint = i;
            }
        }
        Vector2D P = set.get(furthestPoint);
        set.remove(furthestPoint);
        hull.add(insertPosition, P);

        // Determine who's to the left of AP
        ArrayList<Vector2D> leftSetAP = new ArrayList<Vector2D>();
        for (int i = 0; i < set.size(); i++)
        {
            Vector2D M = set.get(i);
            if (pointLocation(A, P, M) == 1)
            {
                leftSetAP.add(M);
            }
        }

        // Determine who's to the left of PB
        ArrayList<Vector2D> leftSetPB = new ArrayList<Vector2D>();
        for (int i = 0; i < set.size(); i++)
        {
            Vector2D M = set.get(i);
            if (pointLocation(P, B, M) == 1)
            {
                leftSetPB.add(M);
            }
        }
        hullSet(A, P, leftSetAP, hull);
        hullSet(P, B, leftSetPB, hull);

    }

    /**
     * Also part of the QuickHull algorithm
     * 
     * @param A
     * @param B
     * @param P
     * @return
     */
    private static int pointLocation(Vector2D A, Vector2D B, Vector2D P)
    {
        double cp1 = (B.x - A.x) * (P.y - A.y) - (B.y - A.y) * (P.x - A.x);
        if (cp1 > 0) return 1;
        else if (cp1 == 0) return 0;
        else return -1;
    }

    /**
     * Grows the given polygon by moving the vertices out by the given distance
     * 
     * @param points
     * @param distance
     * @return
     */
    public static List<Vector2D> growPolygon(List<Vector2D> points,
            double distance) {
        List<Vector2D> result = new ArrayList<Vector2D>(points.size());
        if (points.size() < 3) return points;
        for (int i = 0; i < points.size(); i++) {
            int h = (points.size() + i - 1) % points.size(); // previous point
            int j = (i + 1) % points.size(); // next point;

            Vector2D prev = points.get(h);
            Vector2D current = points.get(i);
            Vector2D next = points.get(j);

            Vector2D v1 = current.minus(prev);
            Vector2D v2 = next.minus(current);

            double acos = (v1.x * v2.x + v1.y * v2.y)
                    / (v1.length() * v2.length());
            if (Math.abs(acos) > 1) {
                acos = Math.signum(acos); // fix for float issue where acos > 1
            }

            double turnAngle = Math.acos(acos);
            double startAngle = Math.atan2(v1.y, v1.x);
            double offsetAngle = startAngle + turnAngle / 2 - Math.PI / 2;

            double offsetDistance = Math.sqrt(Math.pow(
                    distance * Math.tan(turnAngle / 2), 2)
                    + Math.pow(distance, 2));

            Vector2D newPoint = new Vector2D(Math.cos(offsetAngle)
                    * offsetDistance + current.x, Math.sin(offsetAngle)
                    * offsetDistance + current.y);
            result.add(newPoint);
        }
        return result;

    }

    /**
     * Construct a Java Path2D from the given list of Vector2Ds
     * @param positions
     * @return
     */
    public static Path2D listToPath(List<Vector2D> positions) {
        Path2D section = new Path2D.Double();
        section.moveTo(positions.get(0).x, positions.get(0).y);
        for (int i = 1; i < positions.size(); i++) {
            section.lineTo(positions.get(i).x, positions.get(i).y);
        }
        section.lineTo(positions.get(0).x, positions.get(0).y);
        section.closePath();
        return section;
    }

    /**
     * Check if the given parameters overlap
     * @param path
     * @param shape
     * @return
     */
    public static boolean overlapsObstacle(List<Vector2D> path, Shape shape) {
        Path2D section = listToPath(path);
        return overlapsObstacle(section, shape);
    }

    /**
     * Check if the given parameters overlap
     * @param path
     * @param shape
     * @return
     */
    public static boolean overlapsObstacle(List<Vector2D> path,
            List<Vector2D> shape) {
        Path2D section = listToPath(path);
        Path2D shapePath = listToPath(shape);
        return overlapsObstacle(section, shapePath);
    }

    /**
     * Check if the given parameters overlap
     * @param path
     * @param shape
     * @return
     */
    public static boolean overlapsObstacle(Shape path, Shape shape) {
        Area sectionArea = new Area(path);
        sectionArea.intersect(new Area(shape));
        return !sectionArea.isEmpty();
    }

    /**
     * Calculates a regular polygon which approximates a circle from the given center with the given radius and given amount of vertices 
     * @param center
     * @param radius
     * @param vertices
     * @param contain whether or not the circle needs to be contained in the convex polygon, true if the polygon must contain the circle
     * @return
     */
    public static List<Vector2D> approximateCircle(Vector2D center,
            double radius, int vertices, boolean contain) {
        if (contain) {
            radius /= Math.cos(Math.PI / vertices);
        }
        List<Vector2D> result = new ArrayList<Vector2D>(vertices);
        double angle = Math.PI * 2 / vertices;
        for (int i = 0; i < vertices; i++) {
            result.add(new Vector2D(Math.cos(i * angle) * radius + center.x,
                    Math.sin(i * angle) * radius + center.y));
        }
        return result;
    }

    /**
     * Calculates the surface area of the given polygon using the shoelace algorithm
     * @param points
     * @return
     */
    public static double area(List<Vector2D> points) {
        double result = 0;
        for (int i = 0; i < points.size(); i++) {
            Vector2D current = points.get(i);
            Vector2D next = points.get((i + 1) % points.size());
            result += current.x * next.y;
            result -= current.y * next.x;
        }
        result = Math.abs(result / 2);
        return result;
    }
}
