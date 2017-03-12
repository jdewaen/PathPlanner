package pathplanner.boundssolver;

import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;

import org.jenetics.Chromosome;
import org.jenetics.util.ISeq;

import pathplanner.common.Pos2D;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;

// list of pointgenes

public class PolygonChromosome implements Chromosome<PointGene> {

    public final List<PointGene> genes;
     
    public PolygonChromosome(List<PointGene> genes) {
        this.genes = Collections.unmodifiableList(genes);
    }
    
    public static PolygonChromosome fromPos2DList(List<Pos2D> genes) {
        return new PolygonChromosome(genes.stream().map(pos -> PointGene.newInstanceStatic(pos)).collect(Collectors.toList()));
    }
    
    @Override
    public boolean isValid() {
        return true;
    }

    @Override
    public Iterator<PointGene> iterator() {
        return genes.iterator();
    }

    @Override
    public Chromosome<PointGene> newInstance() {
        throw new NotImplementedException();
    }

    @Override
    public PointGene getGene(int arg0) {
        return genes.get(arg0);
    }

    @Override
    public int length() {
        return genes.size();
    }

    @Override
    public Chromosome<PointGene> newInstance(ISeq<PointGene> arg0) {
        return new PolygonChromosome(arg0.asList());
    }

    @Override
    public ISeq<PointGene> toSeq() {
        return ISeq.of(genes);
    }

}
