package test;

import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import pathplanner.PathPlanner;
import pathplanner.PathPlannerFactory;
import pathplanner.PlannerResult;
import pathplanner.ScenarioFactory;
import pathplanner.common.GeometryToolbox;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.ScenarioSegment;
import pathplanner.common.ScenarioSegmentFactory;
import pathplanner.preprocessor.CornerEvent;
import pathplanner.preprocessor.PathNode;
import pathplanner.preprocessor.PathSegment;
import pathplanner.preprocessor.boundssolver.BoundsSolverConfigFactory;


public class GeneticTest extends ParentTest{
    public static final int RUNS = 2;
    
    boolean runBench = true;
    boolean runLeuven = true;
    boolean runSF = true;
    
    private static GeneticStats benchmarkRef = null;
    private static GeneticStats SFRef = null;
    private static GeneticStats leuvenRef = null;

    // 3 cases: more less standard acc
    // 3 cases: more less standard max speed
    // 3 scenarios: benchmark, leuven sf
    @BeforeClass
    public static void setUpBeforeClass() throws Exception {
        
        System.out.println("GENERATING REFERENCES");
        
        
//        System.out.print("BENCHMARK...");
//        Supplier<GeneticStats> benchmarkFunc = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, new BoundsSolverConfigFactory());
//        benchmarkRef = IntStream.range(0, RUNS)
//                .mapToObj(i -> {
//                    GeneticStats stat = benchmarkFunc.get();
//                    System.out.print(" " + i);
//                    return stat;
//                    })
//                .reduce(new GeneticStats(), GeneticStats::add);
//        
//        System.out.println(" DONE");
        
        System.out.print("SF...");
        Supplier<GeneticStats> SFFunc = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, new BoundsSolverConfigFactory());
        SFRef = IntStream.range(0, RUNS)
                .mapToObj(i -> {
                    GeneticStats stat = SFFunc.get();
                    System.out.print(" " + i);
                    return stat;
                    })                .reduce(new GeneticStats(), GeneticStats::add);

        System.out.println(" DONE");
        
        
//        System.out.print("LEUVEN...");
//        Supplier<GeneticStats> leuvenFunc = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, new BoundsSolverConfigFactory());
//        leuvenRef = IntStream.range(0, RUNS)
//                .mapToObj(i -> {
//                    GeneticStats stat = leuvenFunc.get();
//                    System.out.print(" " + i);
//                    return stat;
//                    })                .reduce(new GeneticStats(), GeneticStats::add);
//        
//        System.out.println(" DONE");
        
        System.out.println("REFERENCES DONE");
    }

    @AfterClass
    public static void tearDownAfterClass() throws Exception {}

    @Before
    public void setUp() throws Exception {}

    @After
    public void tearDown() throws Exception {}
    
    // BENCHMARK

//    @Test
//    public void benchmarkLowPoints(){
//        if(!runBench)fail();
//        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::benchmarkLarge, 6);
//        if(!measurePerformance("BENCHMARK LOW POINTS", RUNS, func)) fail();
//    }
//    @Test
//    public void benchmarkMedPoints(){
//        if(!runBench)fail();
//        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::benchmarkLarge, 12);
//        if(!measurePerformance("BENCHMARK MED POINTS", RUNS, func)) fail();
//    }
//    @Test
//    public void benchmarkHighPoints(){
//        if(!runBench)fail();
//        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::benchmarkLarge, 24);
//        if(!measurePerformance("BENCHMARK HIGH POINTS", RUNS, func)) fail();
//    }
    
    // LEUVEN
    
//    @Test
//    public void leuvenLowPoints(){
//        if(!runLeuven)fail();
//        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::leuvenSmall, 6);
//        if(!measurePerformance("LEUVEN LOW POINTS", RUNS, func)) fail();
//    }    
//    @Test
//    public void leuvenMedPoints(){
//        if(!runLeuven)fail();
//        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::leuvenSmall, 12);
//        if(!measurePerformance("LEUVEN MED POINTS", RUNS, func)) fail();
//    }    
//    @Test
//    public void leuvenHighPoints(){
//        if(!runLeuven)fail();
//        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::leuvenSmall, 24);
//        if(!measurePerformance("LEUVEN HIGH POINTS", RUNS, func)) fail();
//    }    
    
    
    // SAN FRANCISCO
    
    @Test
    public void sanFrancisco5Gens(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.maxGens = 5;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 5 GENS", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco15Gens(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.maxGens = 15;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 15 GENS", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco35Gens(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.maxGens = 35;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 35 GENS", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco45Gens(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.maxGens = 45;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 45 GENS", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco5Pop(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.popSize = 5;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 5 POP", RUNS, func, SFRef);
    } 
    
    @Test
    public void sanFrancisco20Pop(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.popSize = 20;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 20 POP", RUNS, func, SFRef);
    } 
    
    @Test
    public void sanFrancisco1NudgeDist(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.maxNudgeDistance = 1;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 1 NUDGE DIST", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco10NudgeDist(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.maxNudgeDistance = 10;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 10 NUDGE DIST", RUNS, func, SFRef);
    } 
    
    @Test
    public void sanFrancisco6MaxPoints(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.maxPoints = 6;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 10 MAX POINTS DIST", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco8MaxPoints(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.maxPoints = 8;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 8 MAX POINTS DIST", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco16MaxPoints(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.maxPoints = 16;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 16 MAX POINTS DIST", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco02AddProb(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.addPointProb = 0.02;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 0.02 ADD PROB", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco20AddProb(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.addPointProb = 0.2;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 0.20 ADD PROB", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco30AddProb(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.addPointProb = 0.3;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 0.30 ADD PROB", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco02RemProb(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.removePointProb = 0.02;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 0.02 REMOVE PROB", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco10RemProb(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.removePointProb = 0.2;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 0.20 REMOVE PROB", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco30RemProb(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.removePointProb = 0.3;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 0.30 REMOVE PROB", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco2MaxAttempts(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.maxAttempts = 2;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 2 MAX ATTEMPTS", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco8MaxAttempts(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.maxAttempts = 8;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 8 MAX ATTEMPTS", RUNS, func, SFRef);
    }
    
    @Test
    public void sanFrancisco25MaxAttempts(){
        if(!runSF)fail();
        BoundsSolverConfigFactory configFact = new BoundsSolverConfigFactory();
        configFact.maxAttempts = 25;
        Supplier<GeneticStats> func = ()-> solveWithConfig(Scenarios::sanFranciscoSmall, configFact);
        measurePerformanceGenetic("SF 25 MAX ATTEMPTS", RUNS, func, SFRef);
    }
//    @Test
//    public void sanFranciscoMedPoints(){
//        if(!runSF)fail();
//        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::sanFranciscoSmall, 12);
//        if(!measurePerformance("SF MED POINTS", RUNS, func)) fail();
//    }    
//    @Test
//    public void sanFranciscoHighPoints(){
//        if(!runSF)fail();
//        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::sanFranciscoSmall, 24);
//        if(!measurePerformance("SF HIGH POINTS", RUNS, func)) fail();
//    }    
    
    
    
    private static GeneticStats solveWithConfig(Supplier<ScenarioFactory> factProvider, BoundsSolverConfigFactory configFactory){
        Scenario scenario = factProvider.get().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        plannerFact.boundsConfig = configFactory.build();
        
        
        PathPlanner planner = plannerFact.build(scenario);
        
        PathNode prePath = planner.cornerHeuristic.solve();        
        List<CornerEvent> corners = planner.cornerHeuristic.generateEvents(prePath);
        List<PathSegment> pathSegments = planner.checkpointGenerator.generateFromPath(prePath, corners);
        List<ScenarioSegmentFactory> scenariosegmentFacts = planner.generateScenarioSegments(pathSegments);
        
        List<Double> times = new ArrayList<Double>(scenariosegmentFacts.size());
        List<Double> areas = new ArrayList<Double>(scenariosegmentFacts.size());
        for(ScenarioSegmentFactory segmentFact : scenariosegmentFacts){
            segmentFact.startVel = new Pos2D(0, 0);
            ScenarioSegment segment = segmentFact.build();
            double time = ((double) segment.generateActiveSet(scenario, configFactory.build())) / 1000;
            double area = GeometryToolbox.area(segment.activeRegion);
            times.add(time);
            areas.add(area);
        }
        return GeneticStats.fromSingle(times, areas);
//        return result;
    }
    
    public boolean measurePerformanceGenetic(String name, int runs, Supplier<GeneticStats> func, GeneticStats ref) {
        System.out.print("RUNNING " + name + "...");
        
        GeneticStats results = IntStream.range(0, runs)
                .mapToObj(i -> {
                    GeneticStats stat = func.get();
                    System.out.print(" " + i);
                    return stat;
                    })
                .reduce(new GeneticStats(), GeneticStats::add);
        System.out.println("");
        System.out.println(results.compareResults(ref));
        System.out.println("----------------");
        return true;
    }

}
