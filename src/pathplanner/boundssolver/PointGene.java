package pathplanner.boundssolver;

import org.jenetics.Gene;

import pathplanner.common.Pos2D;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;



public class PointGene implements Gene<Pos2D, PointGene> {
    public final Pos2D allele;
    
    public PointGene(Pos2D pos){
        allele = pos;
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
        return newInstanceStatic(pos);
    }
    
    
    public static PointGene newInstanceStatic(Pos2D pos) {
        return new PointGene(pos);
    }
    

    



}
