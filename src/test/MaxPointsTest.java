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


public class MaxPointsTest extends ParentTest{
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
    public void benchmarkLowPoints(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::benchmarkLarge, 6);
        if(!measurePerformance("BENCHMARK LOW POINTS", RUNS, func)) fail();
    }
    @Test
    public void benchmarkMedPoints(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::benchmarkLarge, 12);
        if(!measurePerformance("BENCHMARK MED POINTS", RUNS, func)) fail();
    }
    @Test
    public void benchmarkHighPoints(){
        if(!runBench)fail();
        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::benchmarkLarge, 24);
        if(!measurePerformance("BENCHMARK HIGH POINTS", RUNS, func)) fail();
    }
    
    // LEUVEN
    
    @Test
    public void leuvenLowPoints(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::leuvenSmall, 6);
        if(!measurePerformance("LEUVEN LOW POINTS", RUNS, func)) fail();
    }    
    @Test
    public void leuvenMedPoints(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::leuvenSmall, 12);
        if(!measurePerformance("LEUVEN MED POINTS", RUNS, func)) fail();
    }    
    @Test
    public void leuvenHighPoints(){
        if(!runLeuven)fail();
        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::leuvenSmall, 24);
        if(!measurePerformance("LEUVEN HIGH POINTS", RUNS, func)) fail();
    }    
    
    
    // SAN FRANCISCO
    
    @Test
    public void sanFranciscoLowPoints(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::sanFranciscoSmall, 6);
        if(!measurePerformance("SF LOW POINTS", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoMedPoints(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::sanFranciscoSmall, 12);
        if(!measurePerformance("SF MED POINTS", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoHighPoints(){
        if(!runSF)fail();
        Supplier<PlannerResult> func = ()-> solveWithPoints(Scenarios::sanFranciscoSmall, 24);
        if(!measurePerformance("SF HIGH POINTS", RUNS, func)) fail();
    }    
    
    private PlannerResult solveWithPoints(Supplier<ScenarioFactory> factProvider, int points){
        Scenario scenario = factProvider.get().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        CPLEXSolverConfigFactory solverConfigFact = new CPLEXSolverConfigFactory();
        solverConfigFact.maxSpeedPoints = points;
        solverConfigFact.maxAccPoints = points;
        plannerFact.cplexConfig = solverConfigFact.build();
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }

}
