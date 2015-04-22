package rvo2;

import java.util.Vector;

import sim.util.Double3D;

public class Sphere {
	
	final static float M_PI = 3.14159265358979323846f;

	/* Store the goals of the agents. */
	static Vector<Double3D> goals = new Vector<Double3D>();

	static void setupScenario(RVOSimulator sim)
	{
		/* Specify the global time step of the simulation. */
		sim.setTimeStep(1.0f);
		
		/* Specify the default parameters for agents that are subsequently added. */
		sim.setAgentDefaults(15.0f, 10, 10.0f, 1.5f, 2.0f, new Double3D());

		/* Add agents, specifying their start position, and store their goals on the opposite side of the environment. */
		for (float a = 0; a < M_PI; a += 0.1f) {
			final float z = (float) (100.0f * Math.cos(a));
			final float r = (float) (100.0f * Math.sin(a));

			for (int i = 0; i < r / 2.5f; ++i) {
				final float x = (float) (r * Math.cos(i * 2.0f * M_PI / (r / 2.5f)));
				final float y = (float) (r * Math.sin(i * 2.0f * M_PI / (r / 2.5f)));

				sim.addAgent(new Double3D(x, y, z));
				goals.add(sim.getAgentPosition(sim.getNumAgents() - 1).negate());
			}
		}
	}

	static void updateVisualization(RVOSimulator sim)
	{
		/* Output the current global time. */
		System.out.print(sim.getGlobalTime());

		/* Output the position for all the agents. */
		for (int i = 0; i < sim.getNumAgents(); ++i){
			System.out.print(" "+sim.getAgentPosition(i));
		}

		System.out.println();
		System.out.println();
	}

	static void setPreferredVelocities(RVOSimulator sim)
	{
		/* Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal. */
		for (int i = 0; i < sim.getNumAgents(); ++i) {
			Double3D goalVector = goals.get(i).subtract(sim.getAgentPosition(i));

			if (goalVector.lengthSq() > 1.0f) {
				goalVector = goalVector.normalize();
			}

			sim.setAgentPrefVelocity(i, goalVector);
		}
	}

	static boolean reachedGoal(RVOSimulator sim)
	{
		/* Check if all agents have reached their goals. */
		for (int i = 0; i < sim.getNumAgents(); ++i) {
			if (sim.getAgentPosition(i).subtract(goals.get(i)).lengthSq() > 4.0f * sim.getAgentRadius(i) * sim.getAgentRadius(i)) {
				return false;
			}
		}

		return true;
	}


	public static void main(String[] args) {
		/* Create a new simulator instance. */
		RVOSimulator sim = new RVOSimulator();
		
		/* Set up the scenario. */
		setupScenario(sim);

		/* Perform (and manipulate) the simulation. */
		do {
			updateVisualization(sim);
			setPreferredVelocities(sim);
			sim.doStep();
		}
		while (!reachedGoal(sim));
	}

}

