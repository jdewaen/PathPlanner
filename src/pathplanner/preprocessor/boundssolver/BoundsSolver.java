package pathplanner.preprocessor.boundssolver;

import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import org.jenetics.Chromosome;
import org.jenetics.Genotype;
import org.jenetics.engine.Engine;
import org.jenetics.engine.EvolutionResult;
import org.jenetics.engine.EvolutionStatistics;
import org.jenetics.stat.DoubleMomentStatistics;
import org.jenetics.util.Factory;

import pathplanner.common.GeometryToolbox;
import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.Vehicle;


public class BoundsSolver {
//    private static final double PATH_LENGTH_MULTIPLIER = 2;
//    private static final double MUTATION_RATE = 0.9;
//    
//    public Set<Obstacle2DB> activeObstacles = new HashSet<Obstacle2DB>();
//    public List<Pos2D> requiredPoints;
//    public List<Rectangle2D> requiredRects = new ArrayList<Rectangle2D>();
//    private final Factory<Genotype<PointGene>> ENCODING;
    public final Vehicle vehicle;
    public final BoundsSolverConfig config;
//    public final Scenario scenario;
    
    public BoundsSolver(Scenario scenario, BoundsSolverConfig config){
        this.vehicle = scenario.vehicle;
        this.config = config;
    }
    
    private Path2D buildSearchArea(Pos2D center, double pathLength){
        Path2D searchArea =  new Path2D.Double();
        searchArea.moveTo(center.x - pathLength * config.pathLengthMultiplier, center.y - pathLength * config.pathLengthMultiplier);
        searchArea.lineTo(center.x - pathLength * config.pathLengthMultiplier, center.y + pathLength * config.pathLengthMultiplier);
        searchArea.lineTo(center.x + pathLength * config.pathLengthMultiplier, center.y + pathLength * config.pathLengthMultiplier);
        searchArea.lineTo(center.x + pathLength * config.pathLengthMultiplier, center.y - pathLength * config.pathLengthMultiplier);
        searchArea.lineTo(center.x - pathLength * config.pathLengthMultiplier, center.y - pathLength * config.pathLengthMultiplier);
        searchArea.closePath();
        return searchArea;
    }
        
    private Factory<Genotype<PointGene>> buildPopulationFactory(List<Pos2D> seed){
        return () -> {
            List<PointGene> genes = seed.stream().map(pos -> PointGene.newInstanceStatic(pos)).collect(Collectors.toList());
            Chromosome<PointGene> chrom = new PolygonChromosome(genes);
//            System.out.println("INITIAL: " + area(chrom));
            return Genotype.of(chrom);
        };
    }
    
    private List<Rectangle2D> pointsToRectangles(List<Pos2D> points, double size){
        return points.stream().map(p -> pointToRect(p, size)).collect(Collectors.toList());
    }

//    public BoundsSolver(World2D world, Vehicle vehicle, Pos2D center, Set<Obstacle2DB> ignoreRegions, double pathLength, List<Pos2D> requiredPoints, List<Pos2D> seed){
//        this.requiredPoints = requiredPoints;
//        this.vehicle = vehicle;
//        
//        
//        // build search area
//        searchArea =  new Path2D.Double();
//        searchArea.moveTo(center.x - pathLength * PATH_LENGTH_MULTIPLIER, center.y - pathLength * PATH_LENGTH_MULTIPLIER);
//        searchArea.lineTo(center.x - pathLength * PATH_LENGTH_MULTIPLIER, center.y + pathLength * PATH_LENGTH_MULTIPLIER);
//        searchArea.lineTo(center.x + pathLength * PATH_LENGTH_MULTIPLIER, center.y + pathLength * PATH_LENGTH_MULTIPLIER);
//        searchArea.lineTo(center.x + pathLength * PATH_LENGTH_MULTIPLIER, center.y - pathLength * PATH_LENGTH_MULTIPLIER);
//        searchArea.lineTo(center.x - pathLength * PATH_LENGTH_MULTIPLIER, center.y - pathLength * PATH_LENGTH_MULTIPLIER);
//        searchArea.closePath();
        
        
//        // Save active obstacles
//        for(Obstacle2DB obs : world.getObstacles()){
//            if(ignoreRegions.contains(obs)) continue;
//            Area sectionArea = new Area(searchArea);
//            sectionArea.intersect(new Area(obs.shape));
//            if(area(sectionArea) > 0) activeObstacles.add(obs);
//        }
        
        
//        // Generate bounding boxes around start and end points
//        for(Pos2D pos : requiredPoints){
//            requiredRects.add(pointToRect(pos, vehicle.size));
//        }
//        Pos2D[] bounds = rectsBoundingBox(requiredRects);
         
        
//        ENCODING = () -> {
//            List<PointGene> genes = seed.stream().map(pos -> PointGene.newInstanceStatic(pos)).collect(Collectors.toList());
//            return Genotype.of(new PolygonChromosome(genes));
//        };
//    }
    
    public static Rectangle2D pointToRect(Pos2D pos, double size){
        return new Rectangle2D.Double(pos.x - size , pos.y - size , size * 2, size * 2);
    }
    

    
    public static Pos2D[] rectsBoundingBox(List<Rectangle2D> rects){
        double minX = Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxX = - Double.MAX_VALUE;
        double maxY = - Double.MAX_VALUE;
        for(Rectangle2D rect : rects){
            if(rect.getMinX() < minX) minX = rect.getMinX();
            if(rect.getMinY() < minY) minY = rect.getMinY();
            if(rect.getMaxX() > maxX) maxX = rect.getMaxX();
            if(rect.getMaxY() > maxY) maxY = rect.getMaxY();
        }
        Pos2D[] result = new Pos2D[2];
        result[0] = new Pos2D(minX, minY);
        result[1] = new Pos2D(maxX, maxY);
        return result;
    }
    
    public static boolean isValidShape(List<Pos2D> positions, BoundsSolverData data){
        return inSearchArea(positions, data)
                && containsAllRequiredPoints(positions, data)
                && isConvex(positions)
                && !overlapsObstacle(positions, data)
                && !selfIntersects(positions);
    }
    
    public static boolean overlapsObstacle(List<Pos2D> positions, BoundsSolverData data){
        Path2D section = GeometryToolbox.listToPath(positions);
        for(Obstacle2DB obs : data.inActiveObstacles){
            if(GeometryToolbox.overlapsObstacle(section, obs.shape)) return true;
        }
        return false;
    }
    
    public static boolean isConvex(List<Pos2D> positions){
      List<Pos2D> hullPositions = GeometryToolbox.quickHull(new ArrayList<Pos2D>(positions));
      boolean result = (positions.size() == hullPositions.size());
      return result;
    }
    
    private static boolean inSearchArea(List<Pos2D> positions, BoundsSolverData data){
        for(int i = 0; i < positions.size(); i++){
            if(!data.searchArea.contains(positions.get(i).x, positions.get(i).y)) return false;
        }
        return true;
    }
    
    private static boolean containsAllRequiredPoints(List<Pos2D> positions, BoundsSolverData data){
      Path2D section =  GeometryToolbox.listToPath(positions);
      
      for(Pos2D pos : data.requiredPoints){
          if(!section.contains(pos.x, pos.y)) return false;
      }
      for(Rectangle2D rect : data.requiredRects){
          if(!section.contains(rect)) return false;
      }
      return true;
    }
    
    private static boolean selfIntersects(List<Pos2D> positions){
        List<Line2D> lines = new ArrayList<Line2D>();
        for(int i = 0; i < positions.size(); i++){
            Pos2D current = positions.get(i);
            Pos2D next = positions.get((i + 1) % positions.size());
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
    

    
    private static Double fitness(final Genotype<PointGene> gt) {
        return new Double(area(gt.getChromosome()));
    }
    
    private static double area(Chromosome<PointGene> chrom){
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
    
//    private double area(Area area){
//        List<Pos2D> positions = new ArrayList<Pos2D>();
//        
//        PathIterator iter = area.getPathIterator(null);
//        double[] coords = new double[2];
//        
//        while(!iter.isDone()){
//            iter.currentSegment(coords);
//            positions.add(new Pos2D(coords[0], coords[1]));
//            iter.next();
//        }
//        
//        double result = 0;
//        for(int i = 0; i < positions.size(); i++){
//            Pos2D current = positions.get(i);
//            Pos2D next = positions.get((i + 1) % positions.size());
//            result += current.x * next.y;
//            result -= current.y * next.x;
//        }
//        result = Math.abs(result / 2);
//        return result;
//    }
    
    
    public List<Pos2D> solve(Pos2D center, Set<Obstacle2DB> inActiveObstacles, double pathLength, List<Pos2D> requiredPoints, List<Pos2D> seed) {
        
        Path2D searchArea = buildSearchArea(center, pathLength);
        Set<Obstacle2DB> nearbyObstacles = inActiveObstacles.stream().filter(obs -> GeometryToolbox.overlapsObstacle(obs.getVertices(), searchArea)).collect(Collectors.toSet());
        BoundsSolverData data = new BoundsSolverData(
                nearbyObstacles, 
                requiredPoints, 
                pointsToRectangles(requiredPoints, vehicle.size),
                searchArea);
        
        
        final Engine<PointGene, Double> engine = Engine
            .builder(BoundsSolver::fitness, buildPopulationFactory(seed))
            .populationSize(10)
            .alterers(new PolygonMutator(config, data, BoundsSolver::isValidShape))
            .build();
        
        EvolutionStatistics<Double, DoubleMomentStatistics> statistics =
                EvolutionStatistics.ofNumber();
        
        EvolutionResult<PointGene, Double> result = engine.stream()
            .limit(25)
            .peek(statistics)
            .collect(EvolutionResult.toBestEvolutionResult());

//        System.out.println("FINAL: " + result.getBestFitness());
        return result.getBestPhenotype().getGenotype().getChromosome(0).stream().map(gene -> {
            return gene.getAllele();
        }).collect(Collectors.toList());

    }
    
    
    
}
