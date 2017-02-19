package gen;

import static java.lang.Math.pow;
import static org.jenetics.internal.math.random.indexes;

import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.jenetics.AbstractAlterer;
import org.jenetics.Chromosome;
import org.jenetics.Genotype;
import org.jenetics.Phenotype;
import org.jenetics.Population;
import org.jenetics.engine.Engine;
import org.jenetics.engine.EvolutionResult;
import org.jenetics.engine.EvolutionStatistics;
import org.jenetics.internal.util.IntRef;
import org.jenetics.stat.DoubleMomentStatistics;
import org.jenetics.util.Factory;
import org.jenetics.util.ISeq;
import org.jenetics.util.RandomRegistry;

import pathplanner.common.Pos2D;
import pathplanner.common.Region2D;
import pathplanner.common.Scenario;
import pathplanner.common.World2D;


public class AreaSolver {
    private final World2D world;
//    private final Pos2D center = new Pos2D(47, 49);
    private final Pos2D center = new Pos2D(97, 50);
    private final Factory<Genotype<PointGene>> ENCODING;

    public AreaSolver(Scenario scen){
        this.world = scen.world;
        
        ENCODING = () -> {
//            final Random random = RandomRegistry.getRandom();
            int numPoints = 5;
            double chunkSize = Math.PI*2 / numPoints;
            
            List<PointGene> genes = new ArrayList<PointGene>();
            for(int i = 0; i < numPoints; i++){
                double angleOffset = i * chunkSize;
                double angle = randomInRange(angleOffset, angleOffset + chunkSize);
                double distance = randomInRange(0, 50);
                PointGene gene = new PointGene(angle, distance, center);
                while(world.intersectsAnyObstacle(gene.getAllele(), center) 
                        || (i != 0 && !AreaSolver.isValidPoint(genes.get(i-1), gene, world)) // if not first, check with previous
                        || (i == numPoints - 1 && !AreaSolver.isValidPoint(genes.get(0), gene, world))) // if last, check with first as well
                {
                    distance /= 2;
                    gene = new PointGene(angle, distance, center);
                }
                genes.add(gene);
            }
            return Genotype.of(new PolygonChromosome(genes));
        };
    }
    
    public static boolean isValidPoint(PointGene last, PointGene current, World2D world){
        return isValidPoint(last.getAllele(), current.getAllele(), current.center, world);
    }
    
    public static boolean isValidPoint(PointGene last, Pos2D current, World2D world){
        return isValidPoint(last.getAllele(), current, last.center, world);
    }
    
    public static boolean isConvex(List<PointGene> genes){
      ArrayList<Pos2D> positions = (ArrayList<Pos2D>) genes.stream().map(gene -> gene.getAllele()).collect(Collectors.toList());
      List<Pos2D> hullPositions = QuickHull.quickHull(positions);
      boolean result = (genes.size() == hullPositions.size());
      if(!result) System.out.println("convex failed");
      return result;
    }
    
    public static boolean isValidPoint(Pos2D last, Pos2D current, Pos2D center, World2D world){
        Path2D section =  new Path2D.Double();
        section.moveTo(center.x, center.y);
        section.lineTo(last.x, last.y);
        section.lineTo(current.x, current.y);
        section.closePath();
        
        for(Region2D region : world.getRegions()){
            Area sectionArea = new Area(section);
            sectionArea.intersect(new Area(region.shape));
            if(!sectionArea.isEmpty()){
                System.out.println("valid failed");
                return false;
            }
        }
        return true;
    }  
    
    public static double randomInRange(double min, double max){
        Random random = RandomRegistry.getRandom();
        double range = max - min;
        return min + random.nextDouble()*range;
    }
    
    private Double fitness(final Genotype<PointGene> gt) {
        // Calculate fitness from "dynamic" Genotype.
        //System.out.println("Gene count: " + gt.getNumberOfGenes());
        double overlap = 0;
        
        Chromosome<PointGene> chrom = gt.getChromosome();
        
        Path2D section =  new Path2D.Double();
        section.moveTo(chrom.getGene(0).getAllele().x, chrom.getGene(0).getAllele().y);
        for(int i = 1; i < chrom.length(); i++){
            section.lineTo(chrom.getGene(i).getAllele().x, chrom.getGene(i).getAllele().y);
        }
        section.closePath();
        
        for(Region2D region : world.getRegions()){
            Area sectionArea = new Area(section);
            sectionArea.intersect(new Area(region.shape));
            overlap += area(sectionArea);
        }

        overlap *= overlap * overlap;
        double area = area((PolygonChromosome) gt.getChromosome());
        double fitness = area; /// Math.sqrt(gt.getChromosome().length());
//        return new Double(fitness  - overlap);
        return new Double(fitness);
    }
    
    private double area(PolygonChromosome chrom){
        double result = 0;
        for(int i = 0; i < chrom.length(); i++){
            Pos2D current = chrom.getGene(i).getAllele();
            Pos2D next = chrom.getGene((i + 1) % chrom.length()).getAllele();
            result += current.x * next.y;
            result -= current.y * next.x;
        }
        result = Math.abs(result / 2);
        return result;
    }
    
    private double area(Area area){
        List<Pos2D> positions = new ArrayList<Pos2D>();
        
        PathIterator iter = area.getPathIterator(null);
        double[] coords = new double[2];
        
        while(!iter.isDone()){
            iter.currentSegment(coords);
            positions.add(new Pos2D(coords[0], coords[1]));
            iter.next();
        }
        
        double result = 0;
        for(int i = 0; i < positions.size(); i++){
            Pos2D current = positions.get(i);
            Pos2D next = positions.get((i + 1) % positions.size());
            result += current.x * next.y;
            result -= current.y * next.x;
        }
        result = Math.abs(result / 2);
        return result;
    }
    
    private static final class PolygonMutator
    extends AbstractAlterer<PointGene, Double>
{
        
        public final World2D world;
        public final double nudgeDistance = 5;
    public PolygonMutator(double probability, World2D world) {
        super(probability);
        this.world = world;
    }

    @Override
    public int alter(Population<PointGene, Double> population, long generation) {
        final double p = pow(_probability, 1.0/3.0);
        final IntRef alterations = new IntRef(0);

        indexes(RandomRegistry.getRandom(), population.size(), p).forEach(i -> {
            final Phenotype<PointGene, Double> pt = population.get(i);

            final Genotype<PointGene> gt = pt.getGenotype();
            final Genotype<PointGene> mgt = mutate(gt, p, alterations);

            final Phenotype<PointGene, Double> mpt = pt.newInstance(mgt, generation);
            
            population.set(i, mpt);
        });
        return alterations.value;
    }
    
    
    // PER GENOTYPE: ADD/REMOVE CHROMOSOMES
    private Genotype<PointGene> mutate(
            final Genotype<PointGene> genotype,
            final double p,
            final IntRef alterations
        ) {
            final List<Chromosome<PointGene>> chromosomes =
                new ArrayList<>(genotype.toSeq().asList());

            // Add/remove Chromosome to Genotype.
//            final Random random = RandomRegistry.getRandom();
//            final double rd = random.nextDouble();
//            if (rd < 1/3.0) {
//                chromosomes.remove(0);
//            } else if (rd < 2/3.0) {
//                chromosomes.add(chromosomes.get(0).newInstance());
//            }

            alterations.value +=
                indexes(RandomRegistry.getRandom(), chromosomes.size(), p)
                    .map(i -> mutate(chromosomes, i, p))
                    .sum();
            

            return Genotype.of(chromosomes);
        }
    
    
        private int mutate(final List<Chromosome<PointGene>> c, final int i, final double p) {
        final Chromosome<PointGene> chromosome = c.get(i);
        final List<PointGene> genes = new ArrayList<>(chromosome.toSeq().asList());

        final int mutations = mutate(genes, p);
        if (mutations > 0) {
            c.set(i, chromosome.newInstance(ISeq.of(genes)));
        }
        return mutations;
    }
    
    private int mutate(final List<PointGene> genes, final double p) {
        final Random random = RandomRegistry.getRandom();

        // Add/remove Gene from chromosome.
        final double rd = random.nextDouble();
//        if (rd < 1/10.0) {
//            if(genes.size() > 4){
//                genes.remove(random.nextInt(genes.size()));
//            }
//        } else if (rd < 2/10.0) {
//            if(genes.size() < 4){
//                List<PointGene> newGenes = addGene(genes, random.nextInt(genes.size()), nudgeDistance);
//                genes.clear();
//                genes.addAll(newGenes);
//            }
//        }

        int result = (int)indexes(random, genes.size(), p)
                .peek(i -> genes.set(i, genes.get(i).nudge(genes, i, nudgeDistance, world)))
                .count();
        
//        List<PointGene> sortedGenes = sortGenes(genes);
//        ArrayList<Pos2D> positions = (ArrayList<Pos2D>) sortedGenes.stream().map(gene -> gene.getAllele()).collect(Collectors.toList());
//        List<Pos2D> hullPositions = QuickHull.quickHull(positions);
//        Pos2D center = genes.get(0).center;
//        genes.clear();
//        genes.addAll(positions.stream().map(pos -> PointGene.newInstance(pos, center)).collect(Collectors.toList()));
        // fix properly!!
        
//        int result = 1;
        return result;
    }
    
    private List<PointGene> sortGenes(List<PointGene> genes){
        return genes.stream().sorted((g1, g2) -> Double.compare(g1.angleFromCenter, g2.angleFromCenter)).collect(Collectors.toList());
    }
    
    private List<PointGene> addGene(List<PointGene> genes, int index, double maxDistance){
        final Random random = RandomRegistry.getRandom();
        List<PointGene> result = new ArrayList<PointGene>(genes.subList(0, index + 1));
        Pos2D current = genes.get(index % genes.size()).getAllele();
        Pos2D next = genes.get((index + 1) % genes.size()).getAllele();
        Pos2D diff = next.minus(current);
        double dist = random.nextDouble();
        Pos2D newPos = new Pos2D(current.x + diff.x * dist, current.y + diff.y * dist);
        PointGene newGene = genes.get(0).newInstance(newPos);
        //PointGene nudged = newGene.nudge(genes.get((genes.size() + index - 1) % genes.size()), genes.get((index + 1) % genes.size()), maxDistance, world);
        //if(nudged == null) return genes;
        result.add(newGene);
        result.addAll(genes.subList(index + 1, genes.size()));
        return result;
    }
    
}
    public List<Pos2D> solve() {
//        List<PointGene> genes = new ArrayList<PointGene>();
//        Pos2D center = new Pos2D(0, 0);
//        genes.add(PointGene.newInstance(new Pos2D(3, 4), center));
//        genes.add(PointGene.newInstance(new Pos2D(5, 11), center));
//        genes.add(PointGene.newInstance(new Pos2D(12, 8), center));
//        genes.add(PointGene.newInstance(new Pos2D(9, 5), center));
//        genes.add(PointGene.newInstance(new Pos2D(5, 6), center));
//        PolygonChromosome chrom = new PolygonChromosome(genes);
//        System.out.println(area(chrom));
        final Engine<PointGene, Double> engine = Engine
            .builder(this::fitness, ENCODING)
            .populationSize(50)
            .alterers(new PolygonMutator(0.9, world))
            .build();
        
        EvolutionStatistics<Double, DoubleMomentStatistics> statistics =
                EvolutionStatistics.ofNumber();
        
        EvolutionResult<PointGene, Double> result = engine.stream()
            .limit(50)
            .peek(statistics)
            .collect(EvolutionResult.toBestEvolutionResult());

        System.out.println(result.getBestFitness());
        return result.getBestPhenotype().getGenotype().getChromosome(0).stream().map(gene -> {
            return gene.getAllele();
        }).collect(Collectors.toList());

    }
    
}
