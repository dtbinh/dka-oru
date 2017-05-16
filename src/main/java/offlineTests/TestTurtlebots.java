package offlineTests;
import java.util.Arrays;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.meta.spatioTemporal.paths.Map;
import org.metacsp.meta.spatioTemporal.paths.TrajectoryEnvelopeScheduler;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.utility.UI.TrajectoryEnvelopeAnimator;

import com.vividsolutions.jts.geom.Coordinate;

public class TestTurtlebots {
	
	public static void main(String[] args) {
		
		TrajectoryEnvelopeScheduler metaSolver = new TrajectoryEnvelopeScheduler(0, 100000);
		TrajectoryEnvelopeSolver solver = (TrajectoryEnvelopeSolver)metaSolver.getConstraintSolvers()[0];
		Variable[] vars = solver.createVariables(2);
		TrajectoryEnvelope trajEnvelopeRobot1 = (TrajectoryEnvelope)vars[0];
		TrajectoryEnvelope trajEnvelopeRobot2 = (TrajectoryEnvelope)vars[1];

		//Small vehicle, 1.4 (w) x 3.4 (l) centered in the middle
		Coordinate c1 = new Coordinate(1.2, 0.0);
		Coordinate c2 = new Coordinate(0.6, 1.2);
		Coordinate c3 = new Coordinate(-0.6, 1.2);
		Coordinate c4 = new Coordinate(-1.2, 0.0);
		Coordinate c5 = new Coordinate(-0.6, -1.2);
		Coordinate c6 = new Coordinate(0.6, -1.2);

		Trajectory trajRobot1 = new Trajectory("paths/path1.path");
		trajEnvelopeRobot1.setFootprint(c1,c2,c3,c4,c5,c6);
		trajEnvelopeRobot1.setTrajectory(trajRobot1);
		trajEnvelopeRobot1.setRobotID(1);

		Trajectory trajRobot2 = new Trajectory("paths/path3.path");
		trajEnvelopeRobot2.setFootprint(c1,c2,c3,c4,c5,c6);
		trajEnvelopeRobot2.setTrajectory(trajRobot2);
		trajEnvelopeRobot2.setRobotID(2);
		
		System.out.println(trajEnvelopeRobot1 + " has domain " + trajEnvelopeRobot1.getDomain());
		System.out.println(trajEnvelopeRobot2 + " has domain " + trajEnvelopeRobot2.getDomain());
		
		Map map = new Map(null, null);		
		metaSolver.addMetaConstraint(map);
		
		
		ConstraintNetwork refined1 = metaSolver.refineTrajectoryEnvelopes();
		System.out.println("REFINED 1: "+  refined1);

		System.out.println("====================\n== BEFORE SOLVING ==\n====================");
		System.out.println(trajEnvelopeRobot1.getInfo());
		System.out.println(trajEnvelopeRobot2.getInfo());
		
		boolean solved = metaSolver.backtrack();
		System.out.println("Solved? " + solved);
		if (solved) System.out.println("Added resolvers:\n" + Arrays.toString(metaSolver.getAddedResolvers()));

		System.out.println("===================\n== AFTER SOLVING ==\n===================");
		System.out.println(trajEnvelopeRobot1.getInfo());
		System.out.println(trajEnvelopeRobot2.getInfo());

		TrajectoryEnvelopeAnimator tea = new TrajectoryEnvelopeAnimator("This is a test");
		tea.addTrajectoryEnvelopes(trajEnvelopeRobot1, trajEnvelopeRobot2);
		
	}
	
}
