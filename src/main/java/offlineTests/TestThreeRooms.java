package offlineTests;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Vector;
import java.util.logging.Level;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.meta.spatioTemporal.paths.Map;
import org.metacsp.meta.spatioTemporal.paths.TrajectoryEnvelopeScheduler;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint.Type;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.spatial.utility.SpatialRule;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.TrajectoryEnvelopeAnimator;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.vividsolutions.jts.geom.Coordinate;

import dka.timelinePlaning.meta.DKASingleRobotPlanner;
import dka.timelinePlaning.meta.InfoGainOrderingMetaConstraint;

public class TestThreeRooms {
	
	public static void main(String[] args) {
				
		HashMap<Integer, Pose> roomsPositions = new HashMap<Integer, Pose>();
		roomsPositions.put(0, new Pose(0.0, 0.0, 0.0));
		roomsPositions.put(1, new Pose(0.0, 5.0, 0.0));
		roomsPositions.put(2, new Pose(0.0, 10.0, 0.0));
		roomsPositions.put(3, new Pose(0.0, 15.0, 0.0));
		roomsPositions.put(4, new Pose(5.0, 15.0, 0.0));
		roomsPositions.put(5, new Pose(0.0, 25.0, 0.0));
		roomsPositions.put(6, new Pose(10.0, 15.0, 0.0));
		roomsPositions.put(7, new Pose(15.0, 25.0, 0.0));
		roomsPositions.put(8, new Pose(25.0, 5.0, 0.0));
		
		
		HashMap<Integer, Integer> expirations = new HashMap<Integer, Integer>();
		expirations.put(1, 30);
		expirations.put(2, 25);
		expirations.put(3, 70);
		expirations.put(4, 30);
		expirations.put(5, 50);
		expirations.put(6, 60);
		expirations.put(7, 30);
		expirations.put(8, 50);
		
		
		
		DKASingleRobotPlanner metaSolver = new DKASingleRobotPlanner(0, 100000);
		TrajectoryEnvelopeSolver solver = (TrajectoryEnvelopeSolver)metaSolver.getConstraintSolvers()[0];
		Variable[] vars = solver.createVariables(17);
		
		MetaCSPLogging.setLevel(DKASingleRobotPlanner.class, Level.FINEST);
		
		Vector<TrajectoryEnvelope> tes = new Vector<TrajectoryEnvelope>();
		
		//initial case
		TrajectoryEnvelope startTE = (TrajectoryEnvelope)vars[0];
		startTE.getSymbolicVariableActivity().setSymbolicDomain("senseAt","0");
		startTE.setFootprint(getTurtlebotFootprint());
		startTE.setTrajectory(new Trajectory(new Pose[]{roomsPositions.get(0)}));
		startTE.setRobotID(1);
		tes.add(startTE);
		
		AllenIntervalConstraint startTEduration = new AllenIntervalConstraint(Type.Duration, new Bounds(10, 10));
		startTEduration.setFrom(startTE);
		startTEduration.setTo(startTE);
		solver.addConstraint(startTEduration);
		
		
		for (int i = 1; i < vars.length - 1; i=i+2) {
			TrajectoryEnvelope moveTE = (TrajectoryEnvelope)vars[i];
			moveTE.getSymbolicVariableActivity().setSymbolicDomain("moveTo",Integer.toString(i/2 + 1));
			 
			Trajectory traj2 = new Trajectory(new Pose[]{roomsPositions.get((i/2)+1)});
			moveTE.setFootprint(getTurtlebotFootprint());
			moveTE.setTrajectory(traj2);
			moveTE.setRobotID(1);
			tes.add(moveTE);
			
			
			TrajectoryEnvelope senseTE = (TrajectoryEnvelope)vars[i+1];
			senseTE.getSymbolicVariableActivity().setSymbolicDomain("senseAt",Integer.toString(i/2 + 1));
			senseTE.setFootprint(getTurtlebotFootprint());
			senseTE.setTrajectory(traj2);
			senseTE.setRobotID(1);
			tes.add(senseTE);
			
			
			
			AllenIntervalConstraint duration = new AllenIntervalConstraint(Type.Duration, new Bounds(10, 10));
			duration.setFrom(senseTE);
			duration.setTo(senseTE);
			solver.addConstraint(duration);
			
			AllenIntervalConstraint moveMeetsSense = new AllenIntervalConstraint(Type.Meets);
			moveMeetsSense.setFrom(moveTE);
			moveMeetsSense.setTo(senseTE);
			solver.addConstraint(moveMeetsSense);
			

		}
		

		InfoGainOrderingMetaConstraint scheMC = new InfoGainOrderingMetaConstraint(null, null);
		scheMC.setUsage(tes.toArray(new TrajectoryEnvelope[tes.size()]));
		scheMC.setRoomPositions(roomsPositions);
		scheMC.setExpirations(expirations);
		metaSolver.addMetaConstraint(scheMC);
		
		boolean solved = metaSolver.backtrack();
		System.out.println("Solved? " + solved);
		if (solved) System.out.println("Added resolvers:\n" + Arrays.toString(metaSolver.getAddedResolvers()));


		//#####################################################################################################################
		//sort Activity based on the start time for debugging purpose
		HashMap<SymbolicVariableActivity, Long> starttimes = new HashMap<SymbolicVariableActivity, Long>();
		for (int i = 0; i < solver.getRootTrajectoryEnvelopes().length; i++) {
			SymbolicVariableActivity act = solver.getRootTrajectoryEnvelopes()[i].getSymbolicVariableActivity();
			starttimes.put(act, act.getTemporalVariable().getStart().getLowerBound());                       
		}

		//          Collections.sort(starttimes.values());
		starttimes =  sortHashMapByValuesD(starttimes);
		for (SymbolicVariableActivity act : starttimes.keySet()) {
			System.out.println(act + " --> " + starttimes.get(act));
		}
		//#####################################################################################################################

		

		TrajectoryEnvelopeAnimator tea = new TrajectoryEnvelopeAnimator("This is a test");
		tea.setTrajectoryEnvelopes(solver.getConstraintNetwork());		
		//tea.addTrajectoryEnvelopes(tes.toArray(new TrajectoryEnvelope[tes.size()]));
		
	}

	private static LinkedHashMap sortHashMapByValuesD(HashMap passedMap) {
		ArrayList mapKeys = new ArrayList(passedMap.keySet());
		ArrayList mapValues = new ArrayList(passedMap.values());
		Collections.sort(mapValues);
		Collections.sort(mapKeys);

		LinkedHashMap sortedMap =  new LinkedHashMap();

		Iterator valueIt = ((java.util.List<SpatialRule>) mapValues).iterator();
		while (valueIt.hasNext()) {
			long val = (Long) valueIt.next();
			Iterator keyIt = ((java.util.List<SpatialRule>) mapKeys).iterator();

			while (keyIt.hasNext()) {
				SymbolicVariableActivity key = (SymbolicVariableActivity) keyIt.next();
				long comp1 = (Long) passedMap.get(key);
				long comp2 = val;

				if (comp1 == comp2){
					passedMap.remove(key);
					mapKeys.remove(key);
					sortedMap.put(key, val);
					break;
				}
			}
		}
		return sortedMap;
	}
	
	public static Coordinate[] getTurtlebotFootprint(){
		Coordinate[] ret = new Coordinate[6];		
		ret[0] = new Coordinate(1.2, 0.0);
		ret[1] = new Coordinate(0.6, 1.2);
		ret[2] = new Coordinate(-0.6, 1.2);
		ret[3] = new Coordinate(-1.2, 0.0);
		ret[4] = new Coordinate(-0.6, -1.2);
		ret[5] = new Coordinate(0.6, -1.2);
		
		return ret;
	}
}
