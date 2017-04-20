package pathplanner.common;

//This is a java program to find a points in convex hull using quick hull method
//source: Alexander Hrishov's website
//URL: http://www.ahristov.com/tutorial/geometry-games/convex-hull.html


import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class GeometryToolbox
{
public static List<Pos2D> quickHull(List<Pos2D> points)
  {
      ArrayList<Pos2D> convexHull = new ArrayList<Pos2D>();
      if (points.size() < 3)
          return new ArrayList<Pos2D>(points);

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
      Pos2D A = points.get(minPoint);
      Pos2D B = points.get(maxPoint);
      convexHull.add(A);
      convexHull.add(B);
      points.remove(A);
      points.remove(B);

      ArrayList<Pos2D> leftSet = new ArrayList<Pos2D>();
      ArrayList<Pos2D> rightSet = new ArrayList<Pos2D>();

      for (int i = 0; i < points.size(); i++)
      {
          Pos2D p = points.get(i);
          if (pointLocation(A, B, p) == -1)
              leftSet.add(p);
          else if (pointLocation(A, B, p) == 1)
              rightSet.add(p);
      }
      hullSet(A, B, rightSet, convexHull);
      hullSet(B, A, leftSet, convexHull);

      Collections.reverse(convexHull); // Make points CCW
      return convexHull;
  }

  private static double distance(Pos2D A, Pos2D B, Pos2D C)
  {
      double ABx = B.x - A.x;
      double ABy = B.y - A.y;
      double num = ABx * (A.y - C.y) - ABy * (A.x - C.x);
      if (num < 0)
          num = -num;
      return num;
  }

  private static void hullSet(Pos2D A, Pos2D B, ArrayList<Pos2D> set,
          ArrayList<Pos2D> hull)
  {
      int insertPosition = hull.indexOf(B);
      if (set.size() == 0)
          return;
      if (set.size() == 1)
      {
          Pos2D p = set.get(0);
          set.remove(p);
          hull.add(insertPosition, p);
          return;
      }
      double dist = Integer.MIN_VALUE;
      int furthestPoint = -1;
      for (int i = 0; i < set.size(); i++)
      {
          Pos2D p = set.get(i);
          double distance = distance(A, B, p);
          if (distance > dist)
          {
              dist = distance;
              furthestPoint = i;
          }
      }
      Pos2D P = set.get(furthestPoint);
      set.remove(furthestPoint);
      hull.add(insertPosition, P);

      // Determine who's to the left of AP
      ArrayList<Pos2D> leftSetAP = new ArrayList<Pos2D>();
      for (int i = 0; i < set.size(); i++)
      {
          Pos2D M = set.get(i);
          if (pointLocation(A, P, M) == 1)
          {
              leftSetAP.add(M);
          }
      }

      // Determine who's to the left of PB
      ArrayList<Pos2D> leftSetPB = new ArrayList<Pos2D>();
      for (int i = 0; i < set.size(); i++)
      {
          Pos2D M = set.get(i);
          if (pointLocation(P, B, M) == 1)
          {
              leftSetPB.add(M);
          }
      }
      hullSet(A, P, leftSetAP, hull);
      hullSet(P, B, leftSetPB, hull);

  }

  private static int pointLocation(Pos2D A, Pos2D B, Pos2D P)
  {
      double cp1 = (B.x - A.x) * (P.y - A.y) - (B.y - A.y) * (P.x - A.x);
      if (cp1 > 0)
          return 1;
      else if (cp1 == 0)
          return 0;
      else
          return -1;
  }
  
  public static List<Pos2D> growPolygon(List<Pos2D> points, double size){
      List<Pos2D> result = new ArrayList<Pos2D>(points.size());
      if(points.size() < 3) return points;
      for(int i = 0; i < points.size(); i++){
          int h = (points.size() + i - 1) % points.size(); // previous point
          int j = (i + 1) % points.size(); // next point;
          
          Pos2D prev = points.get(h);
          Pos2D current = points.get(i);
          Pos2D next = points.get(j);
          
          Pos2D v1 = current.minus(prev);
          Pos2D v2 = next.minus(current);
          
          double turnAngle = Math.acos((v1.x * v2.x + v1.y * v2.y)/(v1.length() * v2.length()));
          double startAngle = Math.atan2(v1.y, v1.x);
          double offsetAngle = startAngle + turnAngle / 2 - Math.PI / 2;
          
          double offsetDistance = Math.sqrt(Math.pow(size * Math.tan(turnAngle/2), 2) + Math.pow(size, 2));
          
          Pos2D newPoint = new Pos2D(Math.cos(offsetAngle) * offsetDistance + current.x, Math.sin(offsetAngle) * offsetDistance + current.y);
          result.add(newPoint);
      }
      return result;
      
  }
  
  public static Path2D listToPath(List<Pos2D> positions){
      Path2D section =  new Path2D.Double();
      section.moveTo(positions.get(0).x, positions.get(0).y);
      for(int i = 1; i < positions.size(); i++){
          section.lineTo(positions.get(i).x, positions.get(i).y);
      }
      section.lineTo(positions.get(0).x, positions.get(0).y);
      section.closePath();
      return section;
  }
  
  public static boolean overlapsObstacle(List<Pos2D> path, Shape shape){
      Path2D section = listToPath(path);
      return overlapsObstacle(section, shape);
  }
  
  public static boolean overlapsObstacle(List<Pos2D> path, List<Pos2D> shape){
      Path2D section = listToPath(path);
      Path2D shapePath = listToPath(shape);
      return overlapsObstacle(section, shapePath);
  }
  
  public static boolean overlapsObstacle(Shape path, Shape shape){
      Area sectionArea = new Area(path);
      sectionArea.intersect(new Area(shape));
      return !sectionArea.isEmpty();
  }
  
  public static List<Pos2D> approximateCircle(Pos2D center, double radius, int vertices){
      List<Pos2D> result = new ArrayList<Pos2D>(vertices);
      double angle = Math.PI * 2 / vertices;
      for(int i = 0; i < vertices; i++){
          result.add(new Pos2D(Math.cos(i * angle) * radius + center.x, Math.sin(i * angle) * radius + center.y));
      }
      return result;
  }
}