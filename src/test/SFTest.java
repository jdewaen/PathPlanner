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
import pathplanner.common.Scenario;


public class SFTest extends ParentTest{
    public static final int RUNS = 5;

    @BeforeClass
    public static void setUpBeforeClass() throws Exception {}

    @AfterClass
    public static void tearDownAfterClass() throws Exception {}

    @Before
    public void setUp() throws Exception {}

    @After
    public void tearDown() throws Exception {}
    
    @Test
    public void large(){
        if(!measurePerformance("SF LARGE", RUNS, this::solveSingleLarge)) fail();
    }
    
    @Test
    public void small(){
        if(!measurePerformance("SF SMALL", RUNS, this::solveSingleSmall)) fail();
    }
    
    private PlannerResult solveSingleSmall(){
        Scenario scenario = Scenarios.SanFranciscoSmall();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }
    
    private PlannerResult solveSingleLarge(){
        Scenario scenario = Scenarios.SanFranciscoLarge();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }

}
