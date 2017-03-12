package pathplanner.boundssolver;

import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import org.jenetics.Chromosome;
import org.jenetics.Genotype;
import org.jenetics.engine.Engine;
import org.jenetics.engine.EvolutionResult;
import org.jenetics.engine.EvolutionStatistics;
import org.jenetics.stat.DoubleMomentStatistics;
import org.jenetics.util.Factory;

import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.QuickHull;
import pathplanner.common.Vehicle;
import pathplanner.common.World2D;
import pathplanner.preprocessor.PathSegment;


public class BoundsSolver {
    private static final double PATH_LENGTH_MULTIPLIER = 2;
    private static final double MUTATION_RATE = 0.9;
    
    public Set<Obstacle2DB> activeObstacles = new HashSet<Obstacle2DB>();
    public List<Pos2D> requiredPoints;
    public List<Rectangle2D> requiredRects = new ArrayList<Rectangle2D>();
    private final Factory<Genotype<PointGene>> ENCODING;
    public final Path2D searchArea;
    public final Vehicle vehicle;

    public BoundsSolver(World2D world, Vehicle vehicle, Pos2D center, Set<Obstacle2DB> ignoreRegions, double pathLength, List<Pos2D> requiredPoints){
        this.requiredPoints = requiredPoints;
        this.vehicle = vehicle;
        
        
        
        // build search area
        searchArea =  new Path2D.Double();
        searchArea.moveTo(center.x - pathLength * PATH_LENGTH_MULTIPLIER, center.y - pathLength * PATH_LENGTH_MULTIPLIER);
        searchArea.lineTo(center.x - pathLength * PATH_LENGTH_MULTIPLIER, center.y + pathLength * PATH_LENGTH_MULTIPLIER);
        searchArea.lineTo(center.x + pathLength * PATH_LENGTH_MULTIPLIER, center.y + pathLength * PATH_LENGTH_MULTIPLIER);
        searchArea.lineTo(center.x + pathLength * PATH_LENGTH_MULTIPLIER, center.y - pathLength * PATH_LENGTH_MULTIPLIER);
        searchArea.lineTo(center.x - pathLength * PATH_LENGTH_MULTIPLIER, center.y - pathLength * PATH_LENGTH_MULTIPLIER);
        searchArea.closePath();
        
        
        // Save active obstacles
        for(Obstacle2DB obs : world.getObstacles()){
            //if(ignoreRegions.contains(obs)) continue;
            Area sectionArea = new Area(searchArea);
            sectionArea.intersect(new Area(obs.shape));
            if(area(sectionArea) > 0) activeObstacles.add(obs);
        }
        
        
        // Generate bounding boxes around start and end points
        for(Pos2D pos : requiredPoints){
            requiredRects.add(pointToRect(pos, vehicle.size * 2));
        }
        
//        requiredRects.addAll(rects);
//        Pos2D[] bounds = rectsBoundingBox(requiredRects);
         
        
        ENCODING = () -> {
            List<PolygonChromosome> chroms = new ArrayList<PolygonChromosome>();
            List<Pos2D> workingList = new ArrayList<Pos2D>();
            Rectangle2D last = null;
            for(Rectangle2D rect: requiredRects){
                List<Pos2D> tmp = new ArrayList<Pos2D>(workingList);
                tmp.addAll(rectToList(rect));
                tmp = QuickHull.quickHull(tmp);
                if(isValidShape(tmp)){
                    workingList = tmp;
                }else{
                    chroms.add(PolygonChromosome.fromPos2DList(workingList));
                    workingList = rectToList(rect);
//                    workingList.addAll(rectToList(last));
//                    workingList = QuickHull.quickHull(workingList);
                }
                last = rect;
            }
            chroms.add(PolygonChromosome.fromPos2DList(workingList));
//            List<PointGene> genes = new ArrayList<PointGene>();
//            genes.add(PointGene.newInstanceStatic(new Pos2D(bounds[0].x, bounds[0].y)));
//            genes.add(PointGene.newInstanceStatic(new Pos2D(bounds[1].x, bounds[0].y)));
//            genes.add(PointGene.newInstanceStatic(new Pos2D(bounds[1].x, bounds[1].y)));
//            genes.add(PointGene.newInstanceStatic(new Pos2D(bounds[0].x, bounds[1].y)));
            return Genotype.of(chroms);
        };
    }
    
    public static Rectangle2D pointToRect(Pos2D pos, double size){
        return new Rectangle2D.Double(pos.x - size / 2, pos.y - size / 2, size, size);
    }
    
    public static List<Pos2D> rectToList(Rectangle2D rect){
        List<Pos2D> result = new ArrayList<Pos2D>();
        result.add(new Pos2D(rect.getMinX(), rect.getMinY()));
        result.add(new Pos2D(rect.getMaxX(), rect.getMinY()));
        result.add(new Pos2D(rect.getMaxX(), rect.getMaxY()));
        result.add(new Pos2D(rect.getMinX(), rect.getMaxY()));
        return result;
    }
    
    public Path2D listToPath(List<Pos2D> positions){
        Path2D section =  new Path2D.Double();
        section.moveTo(positions.get(0).x, positions.get(0).y);
        for(int i = 1; i < positions.size(); i++){
            section.lineTo(positions.get(i).x, positions.get(i).y);
        }
        section.lineTo(positions.get(0).x, positions.get(0).y);
        section.closePath();
        return section;
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
    
    public boolean isValidShape(List<Pos2D> positions){
        return inSearchArea(positions)
//                && containsAllRequiredPoints(positions)
                && isConvex(positions)
                && !overlapsObstacle(positions)
                && !selfIntersects(positions);
    }
    
    public boolean isValidGenotype(Genotype<PointGene> gt){
        
        List<List<Pos2D>> shapes = gt.stream()
                                    .map(chrom -> chrom.stream()
                                           .map(gene -> gene.getAllele())
                                           .collect(Collectors.toList())
                                        )
                                    .collect(Collectors.toList());
        Optional<List<Pos2D>> individual = shapes.stream().filter(shape -> !isValidShape(shape)).findAny();
        if(individual.isPresent()) 
            return false;
        Area combined = combine(shapes);
        return containsAllRequiredPoints(combined);
    }
    
    public Area combine(List<List<Pos2D>> shapes){
        Area combined = new Area();
        for(List<Pos2D> shape : shapes){
            combined.add(new Area(listToPath(shape)));
        }
        return combined;
    }
        
    public boolean overlapsObstacle(List<Pos2D> positions){
        Path2D section = listToPath(positions);
        for(Obstacle2DB obs : activeObstacles){
            Area sectionArea = new Area(section);
            sectionArea.intersect(new Area(obs.shape));
            
//            List<Pos2D> p = new ArrayList<Pos2D>();
//            PathIterator iter = sectionArea.getPathIterator(null);
//            double[] coords = new double[2];
//            while(!iter.isDone()){
//                iter.currentSegment(coords);
//                p.add(new Pos2D(coords[0], coords[1]));
//                iter.next();
//            }
            
            
            if(!sectionArea.isEmpty()) return true;
        }
        return false;
    }
    
    public boolean isConvex(List<Pos2D> positions){
        for(int i = 0; i < positions.size(); i++){
            Pos2D last = positions.get((i + positions.size() - 1) % positions.size());
            Pos2D current = positions.get(i);
            Pos2D next = positions.get((i + 1) % positions.size());
            if(QuickHull.pointLocation(last, current, next) < 0) return false;
        }
        return true;
//      List<Pos2D> hullPositions = QuickHull.quickHull(new ArrayList<Pos2D>(positions));
//      boolean result = area(new Area(listToPath(positions)))
//              .equals(area(new Area(listToPath(hullPositions))));
//      return result;
    }
    
    private boolean inSearchArea(List<Pos2D> positions){
        for(int i = 0; i < positions.size(); i++){
            if(!searchArea.contains(positions.get(i).x, positions.get(i).y)) return false;
        }
        return true;
    }
    
    private boolean containsAllRequiredPoints(Area area){      
      for(Pos2D pos : requiredPoints){
          if(!area.contains(pos.x, pos.y)) return false;
      }
      for(Rectangle2D rect : requiredRects){
          if(!area.contains(rect)) return false;
      }
      return true;
    }
    
    private boolean containsAllRequiredPoints(List<Pos2D> positions){
        Path2D section =  listToPath(positions);    
        for(Pos2D pos : requiredPoints){
            if(!section.contains(pos.x, pos.y)) return false;
        }
        for(Rectangle2D rect : requiredRects){
            if(!section.contains(rect)) return false;
        }
        return true;
      }
    
    private boolean selfIntersects(List<Pos2D> positions){
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
    

    
    private Double fitness(final Genotype<PointGene> gt) {
        return new Double(gt.stream().map(chrom -> area(chrom)).reduce(0.0, (count, current) -> count + current));
    }
    
    private double area(Chromosome<PointGene> chrom){
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
    
    
    public List<List<Pos2D>> solve() {
        final Engine<PointGene, Double> engine = Engine
            .builder(this::fitness, ENCODING)
            .populationSize(25)
            .alterers(new PolygonMutator(MUTATION_RATE, this))
            .genotypeValidator(gt -> isValidGenotype(gt))
            .build();
        
        EvolutionStatistics<Double, DoubleMomentStatistics> statistics =
                EvolutionStatistics.ofNumber();
        
        EvolutionResult<PointGene, Double> result = engine.stream()
            .limit(200)
            .peek(statistics)
            .collect(EvolutionResult.toBestEvolutionResult());

//        System.out.println(result.getBestFitness());
        
        isValidGenotype(result.getBestPhenotype().getGenotype());
        return result.getBestPhenotype().getGenotype().stream().map( 
                chrom -> chrom.stream()
                    .map(gene -> gene.getAllele())
                    .collect(Collectors.toList())
                ).collect(Collectors.toList());

    }
    
}
