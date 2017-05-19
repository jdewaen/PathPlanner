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


public class TimestepPointsTest extends ParentTest{
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
    public void benchmarkLowFPS(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithFPS(Scenarios::benchmarkLarge, 2);
        if(!measurePerformance("BENCHMARK LOW FPS", RUNS, func)) fail();
    }
    @Test
    public void benchmarkMedFPS(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithFPS(Scenarios::benchmarkLarge, 5);
        if(!measurePerformance("BENCHMARK MED FPS", RUNS, func)) fail();
    }
    @Test
    public void benchmarkHighFPS(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithFPS(Scenarios::benchmarkLarge, 10);
        if(!measurePerformance("BENCHMARK HIGH FPS", RUNS, func)) fail();
    }
    
    // LEUVEN
    
    @Test
    public void leuvenLowFPS(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithFPS(Scenarios::leuvenSmall, 2);
        if(!measurePerformance("LEUVEN LOW FPS", RUNS, func)) fail();
    }    
    @Test
    public void leuvenMedFPS(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithFPS(Scenarios::leuvenSmall, 5);
        if(!measurePerformance("LEUVEN MED FPS", RUNS, func)) fail();
    }    
    @Test
    public void leuvenHighFPS(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithFPS(Scenarios::leuvenSmall, 10);
        if(!measurePerformance("LEUVEN HIGH FPS", RUNS, func)) fail();
    }    
    
    
    // SAN FRANCISCO
    
    @Test
    public void sanFranciscoLowFPS(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithFPS(Scenarios::sanFranciscoSmall, 2);
        if(!measurePerformance("SF LOW FPS", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoMedFPS(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithFPS(Scenarios::sanFranciscoSmall, 5);
        if(!measurePerformance("SF MED FPS", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoHighFPS(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithFPS(Scenarios::sanFranciscoSmall, 10);
        if(!measurePerformance("SF HIGH FPS", RUNS, func)) fail();
    }    
    
    private PlannerResult solveWithFPS(Supplier<ScenarioFactory> factProvider, int fps){
        Scenario scenario = factProvider.get().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        CPLEXSolverConfigFactory solverConfigFact = new CPLEXSolverConfigFactory();
        solverConfigFact.fps = fps;
        plannerFact.cplexConfig = solverConfigFact.build();
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }

}
