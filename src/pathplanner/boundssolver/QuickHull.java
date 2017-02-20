package pathplanner.boundssolver;

//This is a java program to find a points in convex hull using quick hull method
//source: Alexander Hrishov's website
//URL: http://www.ahristov.com/tutorial/geometry-games/convex-hull.html


import java.util.ArrayList;
import java.util.List;

import pathplanner.common.Pos2D;

public class QuickHull
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
}