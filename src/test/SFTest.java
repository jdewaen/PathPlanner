package test;

import static org.junit.Assert.fail;

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


public class SFTest extends ParentTest{
    public static final int RUNS = 50;

    @BeforeClass
    public static void setUpBeforeClass() throws Exception {}

    @AfterClass
    public static void tearDownAfterClass() throws Exception {}

    @Before
    public void setUp() throws Exception {}

    @After
    public void tearDown() throws Exception {}
    
    @Ignore
    @Test
    public void large(){
        if(!measurePerformance("SF LARGE", RUNS, this::solveSingleLarge)) fail();
    }
    
    @Test
    public void small(){
        if(!measurePerformance("SF SMALL", RUNS, this::solveSingleSmall)) fail();
    }
    
    @Ignore
    @Test
    public void smallJerk(){
        if(!measurePerformance("SF SMALL JERK", RUNS, this::solveSingleSmallJerk)) fail();
    }
    
    private PlannerResult solveSingleSmallJerk(){
        ScenarioFactory fact = Scenarios.sanFranciscoSmall();
        fact.vehicle = new Vehicle(10, Double.NaN, 15, 2.5, 100);
        Scenario scenario = fact.build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }
    
    private PlannerResult solveSingleSmall(){
        Scenario scenario = Scenarios.sanFranciscoSmall().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }
    
    private PlannerResult solveSingleLarge(){
        Scenario scenario = Scenarios.sanFranciscoLarge().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }

}
