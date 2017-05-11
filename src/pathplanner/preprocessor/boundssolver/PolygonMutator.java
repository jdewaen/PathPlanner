package pathplanner.preprocessor.boundssolver;

import static java.lang.Math.pow;
import static org.jenetics.internal.math.random.indexes;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.jenetics.AbstractAlterer;
import org.jenetics.Chromosome;
import org.jenetics.Genotype;
import org.jenetics.Phenotype;
import org.jenetics.Population;
import org.jenetics.internal.util.IntRef;
import org.jenetics.util.ISeq;
import org.jenetics.util.RandomRegistry;
import org.omg.CORBA._PolicyStub;

import pathplanner.common.Pos2D;


final class PolygonMutator extends AbstractAlterer<PointGene, Double>
{
    
    private final BoundsSolverConfig config;
    private final BoundsSolverData data;
    private final BiFunction<List<Pos2D>,BoundsSolverData,Boolean> validityCheck;
    
    public PolygonMutator(BoundsSolverConfig config, BoundsSolverData data, BiFunction<List<Pos2D>,BoundsSolverData,Boolean> validityCheck ) {
        super(config.mutationProb);
        this.data = data;
        this.validityCheck = validityCheck;
        this.config = config;
    }

    @Override
    public int alter(Population<PointGene, Double> population, long generation) {
//        population.populationSort();
//        System.out.println(population.get(0).getFitness());
//        final double p = pow(_probability, 1.0/3.0);
        final double p = _probability;
        final IntRef alterations = new IntRef(0);

        IntStream.range(0, population.size()).forEach(i -> {
            final Phenotype<PointGene, Double> pt = population.get(i);

            final Genotype<PointGene> gt = pt.getGenotype();
            final Genotype<PointGene> mgt = mutate(gt, p, alterations);

            final Phenotype<PointGene, Double> mpt = pt.newInstance(mgt, generation);

            population.set(i, mpt);
        });
        return alterations.value;
    }


    private Genotype<PointGene> mutate(
            final Genotype<PointGene> genotype,
            final double p,
            final IntRef alterations
            ) {
        final List<Chromosome<PointGene>> chromosomes =
                new ArrayList<>(genotype.toSeq().asList());

        alterations.value +=
                IntStream.range(0, chromosomes.size())
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
        if (rd < config.removePointProb) {
            if(genes.size() > config.minPoints){
                genes.remove(random.nextInt(genes.size()));
            }
        } else if (rd < config.removePointProb + config.addPointProb) {
            if(genes.size() < config.maxPoints){
                List<PointGene> newGenes = addGene(genes, random.nextInt(genes.size()));
                genes.clear();
                genes.addAll(newGenes);
            }
        }

        int result = (int)indexes(random, genes.size(), p)
                .peek(i -> genes.set(i, nudge(genes.get(i), genes, i)))
                .count();

        return result;
    }

    private List<PointGene> addGene(List<PointGene> genes, int index){
        final Random random = RandomRegistry.getRandom();
        List<PointGene> result = new ArrayList<PointGene>(genes.subList(0, index + 1));
        Pos2D current = genes.get(index % genes.size()).getAllele();
        Pos2D next = genes.get((index + 1) % genes.size()).getAllele();
        Pos2D diff = next.minus(current);
        double dist = random.nextDouble();
        Pos2D newPos = new Pos2D(current.x + diff.x * dist, current.y + diff.y * dist);
        PointGene newGene = genes.get(0).newInstance(newPos);
        result.add(newGene);
        result.addAll(genes.subList(index + 1, genes.size()));
        return result;
    }
    
    private static double randomInRange(double min, double max){
        Random random = RandomRegistry.getRandom();
        double range = max - min;
        return min + random.nextDouble()*range;
    }
    
    private PointGene nudge(PointGene gene, List<PointGene> genes, int i){
        PointGene newGene;
        Pos2D pos;
        ArrayList<PointGene> newGenes = new ArrayList<PointGene>(genes);
        int count = 0;
        do{
            double angle = randomInRange(0, 2*Math.PI);
            double distance = randomInRange(0, config.maxNudgeDistance);
            pos = gene.getAllele().plus(new Pos2D(Math.cos(angle) * distance, Math.sin(angle) * distance));
            newGene = new PointGene(pos);
            count++;
            newGenes.set(i, newGene);
        }while(!validityCheck.apply((ArrayList<Pos2D>) newGenes.stream().map(g -> g.getAllele()).collect(Collectors.toList()), data) 
                && count < config.maxAttempts);
//        
        if(count >= config.maxAttempts){
//            System.out.println("failed");
            return gene;
        }
//        System.out.println("succeeded");
        return newGene;
    }
    

}