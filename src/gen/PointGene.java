package gen;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.jenetics.Gene;
import org.jenetics.util.RandomRegistry;

import pathplanner.common.Pos2D;
import pathplanner.common.World2D;
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
        if(a2 - a1 > 0){
            return (a1 < angleFromCenter && a2 > angleFromCenter);
        }else{
            double a3 = (Math.PI * 2 + angleFromCenter % Math.PI * 2) - Math.PI;
            if (a3 >= 0){
                return (a3 < a2);
            }else{
                return (a3 + Math.PI*2 > a1);
            }
        }
        
    }
    
    public PointGene nudge(List<PointGene> genes, int i, double maxDistance, World2D world){
        int attempts = 5;
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
        }while((!AreaSolver.isValidPoint(previous, newGene, world)
                || !AreaSolver.isValidPoint(newGene, next, world)
                || !AreaSolver.isConvex(newGenes)
                || !newGene.between(previous, next))
                && count < attempts);
        
        if(count >= attempts){
            return this;
        }
        return newGene;
    }


}
