package pathplanner.preprocessor.boundssolver;

import org.jenetics.Gene;

import pathplanner.common.Vector2D;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;



public class PointGene implements Gene<Vector2D, PointGene> {
    public final Vector2D allele;
    
    public PointGene(Vector2D pos){
        allele = pos;
    }
    
    @Override
    public boolean isValid() {
        return true;
    }

    @Override
    public Vector2D getAllele() {
        return allele;
    }

    @Override
    public PointGene newInstance() {
        throw new NotImplementedException();
    }

    @Override
    public PointGene newInstance(Vector2D pos) {
        return newInstanceStatic(pos);
    }
    
    
    public static PointGene newInstanceStatic(Vector2D pos) {
        return new PointGene(pos);
    }
    

    



}
