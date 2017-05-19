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


public class TimeLimitTest extends ParentTest{
    public static final int RUNS = 5;
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
    public void benchmarkLowTimeLimit(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithTimeLimitMultiplier(Scenarios::benchmarkLarge, 1);
        if(!measurePerformance("BENCHMARK LOW TIME LIMIT", RUNS, func)) fail();
    }
    @Test
    public void benchmarkMedTimeLimit(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithTimeLimitMultiplier(Scenarios::benchmarkLarge, 1.5);
        if(!measurePerformance("BENCHMARK MED TIME LIMIT", RUNS, func)) fail();
    }
    @Test
    public void benchmarkHighTimeLimit(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithTimeLimitMultiplier(Scenarios::benchmarkLarge, 2.5);
        if(!measurePerformance("BENCHMARK HIGH TIME LIMIT", RUNS, func)) fail();
    }
    
    // LEUVEN
    
    @Test
    public void leuvenLowTimeLimit(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithTimeLimitMultiplier(Scenarios::leuvenSmall, 1);
        if(!measurePerformance("LEUVEN LOW TIME LIMIT", RUNS, func)) fail();
    }    
    @Test
    public void leuvenMedTimeLimit(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithTimeLimitMultiplier(Scenarios::leuvenSmall, 1.5);
        if(!measurePerformance("LEUVEN MED TIME LIMIT", RUNS, func)) fail();
    }    
    @Test
    public void leuvenHighTimeLimit(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithTimeLimitMultiplier(Scenarios::leuvenSmall, 2.5);
        if(!measurePerformance("LEUVEN HIGH TIME LIMIT", RUNS, func)) fail();
    }    
    
    
    // SAN FRANCISCO
    
    @Test
    public void sanFranciscoLowTimeLimit(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithTimeLimitMultiplier(Scenarios::sanFranciscoSmall, 1);
        if(!measurePerformance("SF LOW TIME LIMIT", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoMedTimeLimit(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithTimeLimitMultiplier(Scenarios::sanFranciscoSmall, 1.5);
        if(!measurePerformance("SF MED TIME LIMIT", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoHighTimeLimit(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithTimeLimitMultiplier(Scenarios::sanFranciscoSmall, 2.5);
        if(!measurePerformance("SF HIGH TIME LIMIT", RUNS, func)) fail();
    }    
    
    private PlannerResult solveWithTimeLimitMultiplier(Supplier<ScenarioFactory> factProvider, double timeLimitMultiplier){
        Scenario scenario = factProvider.get().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        CPLEXSolverConfigFactory solverConfigFact = new CPLEXSolverConfigFactory();
        solverConfigFact.timeLimitMultiplier = timeLimitMultiplier;
        plannerFact.cplexConfig = solverConfigFact.build();
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }

}
