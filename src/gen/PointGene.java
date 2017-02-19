package gen;

import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.jenetics.Gene;

import pathplanner.common.Pos2D;
import pathplanner.common.Region2D;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;

// angle, distance, angle?


public class PointGene implements Gene<Pos2D, PointGene> {
    
    public final double angleFromCenter;
    public final double distance; // 0+
    public final Pos2D center;
    public final Pos2D allele;
    
    public PointGene(double angleFromCenter, double distance, Pos2D center){
        this.angleFromCenter = angleFromCenter;
        this.distance = distance;
        this.center = center;
        allele = center.plus(new Pos2D(Math.cos(angleFromCenter) * distance, Math.sin(angleFromCenter) * distance));
    }
    
    @Override
    public boolean isValid() {
        return true;
    }

    @Override
    public Pos2D getAllele() {
        return allele;
    }

    @Override
    public PointGene newInstance() {
        throw new NotImplementedException();
    }

    @Override
    public PointGene newInstance(Pos2D pos) {
        double distance = pos.distanceFrom(center);
        Pos2D diff = pos.minus(center);
        double angle = Math.atan2(diff.y, diff.x);
        return new PointGene(angle, distance, center);
    }
    
    
    public static PointGene newInstance(Pos2D pos, Pos2D center) {
        double distance = pos.distanceFrom(center);
        Pos2D diff = pos.minus(center);
        double angle = Math.atan2(diff.y, diff.x);
        return new PointGene(angle, distance, center);
    }
    
    public boolean between(PointGene previous, PointGene next){
        double a1 = Math.PI * 2 + previous.angleFromCenter % Math.PI * 2;
        double a2 = Math.PI * 2 + next.angleFromCenter % Math.PI * 2;
        boolean result;
        if(a2 - a1 > 0){
            result = (a1 < angleFromCenter && a2 > angleFromCenter);
        }else{
            double a3 = (Math.PI * 2 + angleFromCenter % Math.PI * 2) - Math.PI;
            if (a3 >= 0){
                result = (a3 < a2);
            }else{
                result = (a3 + Math.PI*2 > a1);
            }
        }
        
//        if(!result) 
//            System.out.println("between failed");
        return result;
        
    }
    
    public PointGene nudge(List<PointGene> genes, int i, double maxDistance, AreaSolver solver){
        int attempts = 15;
//        System.out.println("Nudging...");
        PointGene previous = genes.get((genes.size() + i - 1) % genes.size());
        PointGene next = genes.get((i + 1) % genes.size());
        PointGene newGene;
        Pos2D pos;
        ArrayList<PointGene> newGenes = new ArrayList<PointGene>(genes);
        int count = 0;
        do{
            double angle = AreaSolver.randomInRange(0, 2*Math.PI);
            double distance = AreaSolver.randomInRange(0, maxDistance);
            pos = allele.plus(new Pos2D(Math.cos(angle) * distance, Math.sin(angle) * distance));
            newGene = newInstance(pos);
            count++;
            newGenes.set(i, newGene);
        }while((
                !AreaSolver.inSearchArea(newGene.getAllele(), solver.searchArea)
                || !AreaSolver.containsAllRequiredPoints(newGenes, solver.requiredPoints, solver.requiredRects)
                || !AreaSolver.isConvex(newGenes)
                || !AreaSolver.isValidPoint(previous, newGene, solver.activeRegions)
                || !AreaSolver.isValidPoint(newGene, next, solver.activeRegions)
                || PointGene.selfIntersects(newGenes))
                && count < attempts);
//        
        if(count >= attempts){
//            System.out.println("failed");
            return this;
        }
//        System.out.println("succeeded");
        return newGene;
    }
    
    public static boolean selfIntersects(List<PointGene> genes){
        List<Line2D> lines = new ArrayList<Line2D>();
        for(int i = 0; i < genes.size(); i++){
            Pos2D current = genes.get(i).getAllele();
            Pos2D next = genes.get((i + 1) % genes.size()).getAllele();
            lines.add(new Line2D.Double(current.x, current.y, next.x, next.y));
        }
        
        for(int i = 0; i < lines.size(); i++){
            for(int j = i + 2; j < lines.size(); j++){
                if(i == 0 && j == lines.size() - 1) continue;
                if(lines.get(i).intersectsLine(lines.get(j))) {
//                    System.out.println("selfintersect failed");
                    return true;
                }
            }
        }
        
        return false;
    }


}
