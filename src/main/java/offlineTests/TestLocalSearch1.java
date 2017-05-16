package offlineTests;

import java.util.Calendar;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.time.Bounds;

import aima.core.search.framework.Problem;
import aima.core.search.framework.SearchAgent;
import dka.timelinePlaning.optimizatio.Map;
import dka.timelinePlaning.optimizatio.SequencingFunctionFactory;
import dka.timelinePlaning.optimizatio.SimulatedAnnealingSearch;
import dka.timelinePlaning.optimizatio.TValidityHeuristic;


public class TestLocalSearch1 {

	public static void main(String[] args) {

		HashMap<Integer, Pose> roomsPositions = new HashMap<Integer, Pose>();
		roomsPositions.put(0, new Pose(25.0, 20.0, 0.0));
		roomsPositions.put(1, new Pose(0.0, 5.0, 0.0));
		roomsPositions.put(2, new Pose(0.0, 10.0, 0.0));
		roomsPositions.put(3, new Pose(0.0, 15.0, 0.0));
		roomsPositions.put(4, new Pose(5.0, 15.0, 0.0));
		roomsPositions.put(5, new Pose(0.0, 25.0, 0.0));
		roomsPositions.put(6, new Pose(10.0, 15.0, 0.0));
		roomsPositions.put(7, new Pose(15.0, 25.0, 0.0));
		roomsPositions.put(8, new Pose(25.0, 5.0, 0.0));
		roomsPositions.put(9, new Pose(0.0, 0.0, 0.0));
		roomsPositions.put(10, new Pose(11.0, 25.0, 0.0));
		roomsPositions.put(11, new Pose(35.0, 10.0, 0.0));
		roomsPositions.put(12, new Pose(40.0, 15.0, 0.0));
		roomsPositions.put(13, new Pose(45.0, 0.0, 0.0));
		roomsPositions.put(14, new Pose(40.0, 25.0, 0.0));
		roomsPositions.put(15, new Pose(10.0, 15.0, 0.0));
		roomsPositions.put(16, new Pose(55.0, 25.0, 0.0));
				
		
		HashMap<Integer, Bounds> expirations = new HashMap<Integer, Bounds>();
//		expirations.put(1, 30);
//		expirations.put(2, 25);
//		expirations.put(3, 70);
//		expirations.put(4, 30);
//		expirations.put(5, 50);
//		expirations.put(6, 60);
//		expirations.put(7, 30);
//		expirations.put(8, 50);

		
		expirations.put(1, new Bounds(40, 60));
		expirations.put(2, new Bounds(25, 45));
		expirations.put(3, new Bounds(70, 90));
		expirations.put(4, new Bounds(70, 90));
		expirations.put(5, new Bounds(50, 70));
		expirations.put(6, new Bounds(60, 80));
		expirations.put(7, new Bounds(130, 150));
		expirations.put(8, new Bounds(50, 70));
		
		expirations.put(9, new Bounds(170, 190));
		expirations.put(10, new Bounds(160, 180));
		expirations.put(11, new Bounds(170, 190));
		expirations.put(12, new Bounds(80, 100));
		expirations.put(13, new Bounds(190, 210));
		expirations.put(14, new Bounds(105, 125));
		expirations.put(15, new Bounds(100, 120));
		expirations.put(16, new Bounds(70, 90));

		
		
		int[] roomOrder = new int[16];
		for (int i = 0; i < roomOrder.length; i++) {
			roomOrder[i] = i +1;
		}
		
		long start = Calendar.getInstance().getTimeInMillis();

		Map map = new Map(roomOrder);
		try {
			Problem problem = new Problem(map,
					SequencingFunctionFactory.getIActionsFunction(),
					SequencingFunctionFactory.getResultFunction(),
					null);

			TValidityHeuristic ttcheur = new TValidityHeuristic();
			ttcheur.setExpirations(expirations);
			ttcheur.setRoomPositions(roomsPositions);
			
			SimulatedAnnealingSearch search = new SimulatedAnnealingSearch(ttcheur);
			SearchAgent agent = new SearchAgent(problem, search);

			System.out.println();
			System.out.println("Final State=\n" + search.getLastSearchState());
			System.out.println("************************************************");
			System.out.println("Best State=\n " + search.getBestState());
			System.out.println("bestValue: " + search.getBestValue());


		} catch (Exception e) {
			e.printStackTrace();
		}

		long end = Calendar.getInstance().getTimeInMillis();
		
		System.out.println("COMPUTATION TIME: " +(end - start)/1000);
	}

}
