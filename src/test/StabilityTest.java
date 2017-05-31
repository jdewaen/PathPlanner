package test;

import static org.junit.Assert.fail;

import java.util.function.Supplier;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import pathplanner.PathPlanner;
import pathplanner.PathPlannerFactory;
import pathplanner.PlannerResult;
import pathplanner.ScenarioFactory;
import pathplanner.common.Scenario;
import pathplanner.common.Vehicle;
import pathplanner.milpplanner.CPLEXSolverConfigFactory;


public class StabilityTest extends ParentTest{
    public static final int RUNS = 50;
    boolean runBench = true;
    boolean runLeuven = true;
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
    public void benchmarkNoOverlap(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlap(Scenarios::benchmarkLarge, 1);
        if(!measurePerformance("BENCHMARK NO OVERLAP", RUNS, func)) fail();
    }
    @Test
    public void benchmarkOverlap(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlap(Scenarios::benchmarkLarge, 5);
        if(!measurePerformance("BENCHMARK OVERLAP", RUNS, func)) fail();
    }
    
    // LEUVEN
    
    @Test
    public void leuvenNoOverlap(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlap(Scenarios::leuvenSmall, 1);
        if(!measurePerformance("LEUVEN NO OVERLAP", RUNS, func)) fail();
    }
    
    
    @Test
    public void leuvenOverlap(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlap(Scenarios::leuvenSmall, 5);
        if(!measurePerformance("LEUVEN OVERLAP", RUNS, func)) fail();
    }       
    
    
    // SAN FRANCISCO
    
    @Test
    public void sanFranciscoNoOverlap(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlap(Scenarios::sanFranciscoSmall, 1);
        if(!measurePerformance("SF NO OVERLAP", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoOverlap(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithOverlap(Scenarios::sanFranciscoSmall, 5);
        if(!measurePerformance("SF OVERLAP", RUNS, func)) fail();
    }     
    
    private PlannerResult solveWithOverlap(Supplier<ScenarioFactory> factProvider, int overlap){
        Scenario scenario = factProvider.get().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        plannerFact.overlap = overlap;
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }

}
