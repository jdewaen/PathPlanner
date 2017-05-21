package test;

import static org.junit.Assert.fail;

import java.util.function.Supplier;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import pathplanner.PathPlanner;
import pathplanner.PathPlannerFactory;
import pathplanner.PlannerResult;
import pathplanner.ScenarioFactory;
import pathplanner.common.Scenario;
import pathplanner.common.Vehicle;
import pathplanner.milpplanner.CPLEXSolverConfigFactory;
import pathplanner.preprocessor.segments.SegmentGeneratorConfigFactory;


public class ApproachMarginTest extends ParentTest{
    public static final int RUNS = 5;
    boolean runBench = true;
    boolean runLeuven = false;
    boolean runSF = true;

    // 3 cases: more less standard acc
    // 3 cases: more less standard max speed
    // 3 scenarios: benchmark, leuven sf
    @BeforeClass
    public static void setUpBeforeClass() throws Exception {
        
    }

    @AfterClass
    public static void tearDownAfterClass() throws Exception {}

    @Before
    public void setUp() throws Exception {}

    @After
    public void tearDown() throws Exception {}
    
    // BENCHMARK

    @Test
    public void benchmarkNoOverlapLowMargin(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlapAndMargin(Scenarios::benchmarkLarge, 1, 1.1);
        if(!measurePerformance("BENCHMARK NO OVERLAP LOW MARGIN", RUNS, func)) fail();
    }
    @Test
    public void benchmarkNoOverlapMedMargin(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlapAndMargin(Scenarios::benchmarkLarge, 1, 2);
        if(!measurePerformance("BENCHMARK NO OVERLAP MED MARGIN", RUNS, func)) fail();
    }
    @Test
    public void benchmarkNoOverlapHighMargin(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlapAndMargin(Scenarios::benchmarkLarge, 1, 3);
        if(!measurePerformance("BENCHMARK NO OVERLAP HIGH MARGIN", RUNS, func)) fail();
    }
    @Test
    public void benchmarkOverlapLowMargin(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlapAndMargin(Scenarios::benchmarkLarge, 5, 1.1);
        if(!measurePerformance("BENCHMARK OVERLAP LOW MARGIN", RUNS, func)) fail();
    }
    
    // LEUVEN
    
    @Test
    public void leuvenNoOverlapLowMargin(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlapAndMargin(Scenarios::leuvenSmall, 1, 1.1);
        if(!measurePerformance("LEUVEN NO OVERLAP LOW MARGIN", RUNS, func)) fail();
    }    
    @Test
    public void leuvenNoOverlapMedMargin(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlapAndMargin(Scenarios::leuvenSmall, 1, 2);
        if(!measurePerformance("LEUVEN NO OVERLAP MED MARGIN", RUNS, func)) fail();
    }       
    @Test
    public void leuvenNoOverlapHighMargin(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlapAndMargin(Scenarios::leuvenSmall, 1, 3);
        if(!measurePerformance("LEUVEN NO OVERLAP HIGH MARGIN", RUNS, func)) fail();
    }       
    @Test
    public void leuvenOverlapLowMargin(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlapAndMargin(Scenarios::leuvenSmall, 5, 1.1);
        if(!measurePerformance("LEUVEN OVERLAP LOW MARGIN", RUNS, func)) fail();
    }       
     
    
    // SAN FRANCISCO
    
    @Test
    public void sanFranciscoNoOverlapLowMargin(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlapAndMargin(Scenarios::sanFranciscoSmall, 1, 1.1);
        if(!measurePerformance("SF NO OVERLAP LOW MARGIN", RUNS, func)) fail();
    }  
    @Test
    public void sanFranciscoNoOverlapMedMargin(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlapAndMargin(Scenarios::sanFranciscoSmall, 1, 2);
        if(!measurePerformance("SF NO OVERLAP MED MARGIN", RUNS, func)) fail();
    }  
    @Test
    public void sanFranciscoNoOverlapHighMargin(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlapAndMargin(Scenarios::sanFranciscoSmall, 1, 3);
        if(!measurePerformance("SF NO OVERLAP HIGH MARGIN", RUNS, func)) fail();
    }  
    @Test
    public void sanFranciscoOverlapLowMargin(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlapAndMargin(Scenarios::sanFranciscoSmall, 5, 1.1);
        if(!measurePerformance("SF OVERLAP LOW MARGIN", RUNS, func)) fail();
    }     
    
    private PlannerResult solveWithOverlapAndMargin(Supplier<ScenarioFactory> factProvider, int overlap, double approachMargin){
        Scenario scenario = factProvider.get().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        SegmentGeneratorConfigFactory segmFact = new SegmentGeneratorConfigFactory();
        segmFact.approachMargin = approachMargin;
        plannerFact.segmentConfig = segmFact.build();
        plannerFact.overlap = overlap;
//        plannerFact.verbose = true;
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }

}
