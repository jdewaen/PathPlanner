package test;

import java.util.function.Supplier;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import pathplanner.ScenarioFactory;
import pathplanner.common.Scenario;


public class EdgesTest extends ParentTest{
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
        printEdgeCount("BENCHMARK SMALL", Scenarios::benchmarkSmall);
    }
    
    @Ignore
    @Test
    public void benchmarkSmallFlat(){
        printEdgeCount("BENCHMARK SMALL FLAT", Scenarios::flatBenchmarkSmall);
    }

    @Ignore
    @Test
    public void benchmarkSmallDiag(){
        printEdgeCount("BENCHMARK SMALL DIAG",Scenarios::flatBenchmarkDiagSmall);
    }

    @Test
    public void benchmarkLarge(){
        printEdgeCount("BENCHMARK LARGE", Scenarios::benchmarkLarge);
    }
    
    @Test
    public void spiral(){
        printEdgeCount("SPIRAL", Scenarios::spiral);
    }
        
    @Test
    public void leuvenSmall1(){
        printEdgeCount("LEUVEN SMALL 1", Scenarios::leuvenSmall);
    }
    
    @Test
    public void leuvenSmall2(){
        printEdgeCount("LEUVEN SMALL 2", Scenarios::leuvenSmallAlternate);
    }
    
    @Test
    public void leuvenLarge1(){
        printEdgeCount("LEUVEN LARGE 1", Scenarios::leuvenLarge);
    }

    @Test
    public void sanFranciscoSmall1(){
        printEdgeCount("SF SMALL 1", Scenarios::sanFranciscoSmall);
    }
    
    @Test
    public void sanFranciscoSmall2(){
        printEdgeCount("SF SMALL 2", Scenarios::sanFranciscoSmallAlternate);
    }
    
    @Test
    public void sanFranciscoLarge1(){
        printEdgeCount("SF LARGE 1", Scenarios::sanFranciscoLarge);
    }
    
    
    private void printEdgeCount(String text, Supplier<ScenarioFactory> factProvider){
        Scenario scen = factProvider.get().build();
        int count = (int) scen.world.getObstacles().stream()
                .flatMap(obs -> obs.getVertices().stream())
                .count();
        System.out.println(text + ": " + count);
    }

}
