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


public class GeneralPerformanceTest extends ParentTest{
    public static final int RUNS = 5;

    @BeforeClass
    public static void setUpBeforeClass() throws Exception {
        
    }

    @AfterClass
    public static void tearDownAfterClass() throws Exception {}

    @Before
    public void setUp() throws Exception {}

    @After
    public void tearDown() throws Exception {}
    
    @Test
    public void benchmarkSmall(){
        Supplier<PlannerResult> func = ()-> solve(Scenarios::benchmarkSmall);
        if(!measurePerformance("BENCHMARK SMALL", RUNS, func)) fail();
    }
    
    @Ignore
    @Test
    public void benchmarkSmallFlat(){
        Supplier<PlannerResult> func = ()-> solve(Scenarios::flatBenchmarkSmall);
        if(!measurePerformance("BENCHMARK SMALL FLAT", RUNS, func)) fail();
    }

    @Ignore
    @Test
    public void benchmarkSmallDiag(){
        Supplier<PlannerResult> func = ()-> solve(Scenarios::flatBenchmarkDiagSmall);
        if(!measurePerformance("BENCHMARK SMALL DIAG", RUNS, func)) fail();
    }

    @Test
    public void benchmarkLarge(){
        Supplier<PlannerResult> func = ()-> solve(Scenarios::benchmarkLarge);
        if(!measurePerformance("BENCHMARK LARGE", RUNS, func)) fail();
    }
    
    @Test
    public void spiral(){
        Supplier<PlannerResult> func = ()-> solve(Scenarios::spiral);
        if(!measurePerformance("SPIRAL", RUNS, func)) fail();
    }
        
    @Test
    public void leuvenSmall1(){
        Supplier<PlannerResult> func = ()-> solve(Scenarios::leuvenSmall);
        if(!measurePerformance("LEUVEN SMALL 1", RUNS, func)) fail();
    }
    
    @Test
    public void leuvenSmall2(){
        Supplier<PlannerResult> func = ()-> solve(Scenarios::leuvenSmallAlternate);
        if(!measurePerformance("LEUVEN SMALL 2", RUNS, func)) fail();
    }
    
    @Test
    public void leuvenLarge1(){
        Supplier<PlannerResult> func = ()-> solve(Scenarios::leuvenLarge);
        if(!measurePerformance("LEUVEN LARGE 1", RUNS, func)) fail();
    }

    @Test
    public void sanFranciscoSmall1(){
        Supplier<PlannerResult> func = ()-> solve(Scenarios::sanFranciscoSmall);
        if(!measurePerformance("SF SMALL 1", RUNS, func)) fail();
    }
    
    @Test
    public void sanFranciscoSmall2(){
        Supplier<PlannerResult> func = ()-> solve(Scenarios::sanFranciscoSmallAlternate);
        if(!measurePerformance("SF SMALL 2", RUNS, func)) fail();
    }
    
    @Test
    public void sanFranciscoLarge1(){
        Supplier<PlannerResult> func = ()-> solve(Scenarios::sanFranciscoLarge);
        if(!measurePerformance("SF LARGE 1", RUNS, func)) fail();
    }

}
