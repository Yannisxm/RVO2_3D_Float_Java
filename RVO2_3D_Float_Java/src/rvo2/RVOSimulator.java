package rvo2;

import java.util.Vector;

import sim.util.Double3D;

public class RVOSimulator {
	/**
	 * \brief   Error value.	 *
	 * A value equal to the largest unsigned integer, which is returned in case of an error by functions in RVO::RVOSimulator.
	 */
	final int RVO_ERROR = Integer.MAX_VALUE;

	private float globalTime_;
	float timeStep_;
	
	private Agent ptrdefaultAgent_=null;
	KdTree ptrkdTree_=null;	
	Vector<Agent> agents_=new Vector<Agent>();
	
	/**
	 * \brief   finalructs a simulator instance.
	 */
	public RVOSimulator()
	{
		globalTime_=0.0f;
		timeStep_=0.0f;
		ptrkdTree_ = new KdTree(this);
	}
	/**
	 * \brief   finalructs a simulator instance and sets the default properties for any new agent that is added.
	 * \param   timeStep      The time step of the simulation. Must be positive.
	 * \param   neighborDist  The default maximum distance (center point to center point) to other agents a new agent takes into account in the navigation. The larger this number, the longer he running time of the simulation. If the number is too low, the simulation will not be safe. Must be non-negative.
	 * \param   maxNeighbors  The default maximum number of other agents a new agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe.
	 * \param   timeHorizon   The default minimum amount of time for which a new agent's velocities that are computed by the simulation are safe with respect to other agents. The larger this number, the sooner an agent will respond to the presence of other agents, but the less freedom the agent has in choosing its velocities. Must be positive.
	 * \param   radius        The default radius of a new agent. Must be non-negative.
	 * \param   maxSpeed      The default maximum speed of a new agent. Must be non-negative.
	 * \param   velocity      The default initial three-dimensional linear velocity of a new agent (optional).
	 */
	public RVOSimulator(float timeStep, float neighborDist, int maxNeighbors, float timeHorizon, float radius, float maxSpeed, Double3D velocity)
	{
		globalTime_=0.0f;
		timeStep_=timeStep;
		ptrkdTree_ = new KdTree(this);
		ptrdefaultAgent_ = new Agent(this);

		ptrdefaultAgent_.maxNeighbors_ = maxNeighbors;
		ptrdefaultAgent_.maxSpeed_ = maxSpeed;
		ptrdefaultAgent_.neighborDist_ = neighborDist;
		ptrdefaultAgent_.radius_ = radius;
		ptrdefaultAgent_.timeHorizon_ = timeHorizon;
		ptrdefaultAgent_.velocity_ = velocity;
	}
	
	/**
	 * \brief   Sets the default properties for any new agent that is added.
	 * \param   neighborDist  The default maximum distance (center point to center point) to other agents a new agent takes into account in the navigation. The larger this number, the longer he running time of the simulation. If the number is too low, the simulation will not be safe. Must be non-negative.
	 * \param   maxNeighbors  The default maximum number of other agents a new agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe.
	 * \param   timeHorizon   The default minimum amount of time for which a new agent's velocities that are computed by the simulation are safe with respect to other agents. The larger this number, the sooner an agent will respond to the presence of other agents, but the less freedom the agent has in choosing its velocities. Must be positive.
	 * \param   radius        The default radius of a new agent. Must be non-negative.
	 * \param   maxSpeed      The default maximum speed of a new agent. Must be non-negative.
	 * \param   velocity      The default initial three-dimensional linear velocity of a new agent (optional).
	 */
	public void setAgentDefaults(float neighborDist, int maxNeighbors, float timeHorizon, float radius, float maxSpeed, Double3D velocity)
	{
		if (ptrdefaultAgent_ == null) {
			ptrdefaultAgent_ = new Agent(this);
		}

		ptrdefaultAgent_.maxNeighbors_ = maxNeighbors;
		ptrdefaultAgent_.maxSpeed_ = maxSpeed;
		ptrdefaultAgent_.neighborDist_ = neighborDist;
		ptrdefaultAgent_.radius_ = radius;
		ptrdefaultAgent_.timeHorizon_ = timeHorizon;
		ptrdefaultAgent_.velocity_ = velocity;
	}

	/**
	 * \brief   Adds a new agent with default properties to the simulation.
	 * \param   position  The three-dimensional starting position of this agent.
	 * \return  The number of the agent, or RVO::RVO_ERROR when the agent defaults have not been set.
	 */
	public int addAgent(Double3D position)
	{
		if (ptrdefaultAgent_ == null) {
			return RVO_ERROR;
		}

		Agent agent = new Agent(this);

		agent.position_ = position;
		agent.maxNeighbors_ = ptrdefaultAgent_.maxNeighbors_;
		agent.maxSpeed_ = ptrdefaultAgent_.maxSpeed_;
		agent.neighborDist_ = ptrdefaultAgent_.neighborDist_;
		agent.radius_ = ptrdefaultAgent_.radius_;
		agent.timeHorizon_ = ptrdefaultAgent_.timeHorizon_;
		agent.velocity_ = ptrdefaultAgent_.velocity_;

		agent.id_ = agents_.size();

		agents_.add(agent);

		return agents_.size() - 1;
	}

	/**
	 * \brief   Adds a new agent to the simulation.
	 * \param   position      The three-dimensional starting position of this agent.
	 * \param   neighborDist  The maximum distance (center point to center point) to other agents this agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe. Must be non-negative.
	 * \param   maxNeighbors  The maximum number of other agents this agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe.
	 * \param   timeHorizon   The minimum amount of time for which this agent's velocities that are computed by the simulation are safe with respect to other agents. The larger this number, the sooner this agent will respond to the presence of other agents, but the less freedom this agent has in choosing its velocities. Must be positive.
	 * \param   radius        The radius of this agent. Must be non-negative.
	 * \param   maxSpeed      The maximum speed of this agent. Must be non-negative.
	 * \param   velocity      The initial three-dimensional linear velocity of this agent (optional).
	 * \return  The number of the agent.
	 */
	public int addAgent(Double3D position, float neighborDist, int maxNeighbors, float timeHorizon, float radius, float maxSpeed, Double3D velocity)
	{
		Agent agent = new Agent(this);

		agent.position_ = position;
		agent.maxNeighbors_ = maxNeighbors;
		agent.maxSpeed_ = maxSpeed;
		agent.neighborDist_ = neighborDist;
		agent.radius_ = radius;
		agent.timeHorizon_ = timeHorizon;
		agent.velocity_ = velocity;

		agent.id_ = agents_.size();

		agents_.add(agent);

		return agents_.size() - 1;
	}
	
	/**
	 * \brief   Removes an agent from the simulation.
	 * \param   agentNo  The number of the agent that is to be removed.
	 * \note    After the removal of the agent, the agent that previously had number getNumAgents() - 1 will now have number agentNo.
	 */
	public void removeAgent(int agentNo){
		agents_.set(agentNo,agents_.lastElement());
		agents_.remove(agents_.size()-1);
	}
	
	/**
	 * \brief   Lets the simulator perform a simulation step and updates the three-dimensional position and three-dimensional velocity of each agent.
	 */
	public void doStep()
	{
		ptrkdTree_.buildAgentTree();
		
		for (int i = 0; i <agents_.size(); ++i) 
		{
			agents_.get(i).computeNeighbors();
			agents_.get(i).computeNewVelocity();
		}

		for (int i = 0; i < agents_.size(); ++i) 
		{
			agents_.get(i).update();
		}

		globalTime_ += timeStep_;
	}
	/**
	 * \brief   Returns the specified agent neighbor of the specified agent.
	 * \param   agentNo     The number of the agent whose agent neighbor is to be retrieved.
	 * \param   neighborNo  The number of the agent neighbor to be retrieved.
	 * \return  The number of the neighboring agent.
	 */
	public int getAgentAgentNeighbor(int agentNo, int neighborNo)
	{
		return agents_.get(agentNo).agentNeighbors_.get(neighborNo).getSecond().id_;
	}
	/**
	 * \brief   Returns the maximum neighbor count of a specified agent.
	 * \param   agentNo  The number of the agent whose maximum neighbor count is to be retrieved.
	 * \return  The present maximum neighbor count of the agent.
	 */
	public int getAgentMaxNeighbors(int agentNo){
		return agents_.get(agentNo).maxNeighbors_;
	}

	/**
	 * \brief   Returns the maximum speed of a specified agent.
	 * \param   agentNo  The number of the agent whose maximum speed is to be retrieved.
	 * \return  The present maximum speed of the agent.
	 */
	public float getAgentMaxSpeed(int agentNo){
		return agents_.get(agentNo).maxSpeed_;
	}
	/**
	 * \brief   Returns the maximum neighbor distance of a specified agent.
	 * \param   agentNo  The number of the agent whose maximum neighbor distance is to be retrieved.
	 * \return  The present maximum neighbor distance of the agent.
	 */
	public float getAgentNeighborDist(int agentNo){
		return agents_.get(agentNo).neighborDist_;
	}
	/**
	 * \brief   Returns the count of agent neighbors taken into account to compute the current velocity for the specified agent.
	 * \param   agentNo  The number of the agent whose count of agent neighbors is to be retrieved.
	 * \return  The count of agent neighbors taken into account to compute the current velocity for the specified agent.
	 */
	public int getAgentNumAgentNeighbors(int agentNo){
		return agents_.get(agentNo).agentNeighbors_.size();
	}
	/**
	 * \brief   Returns the count of ORCA finalraints used to compute the current velocity for the specified agent.
	 * \param   agentNo  The number of the agent whose count of ORCA finalraints is to be retrieved.
	 * \return  The count of ORCA finalraints used to compute the current velocity for the specified agent.
	 */
	public int getAgentNumORCAPlanes(int agentNo){
		return agents_.get(agentNo).orcaPlanes_.size();
	}

	/**
	 * \brief   Returns the specified ORCA finalraint of the specified agent.
	 * \param   agentNo  The number of the agent whose ORCA finalraint is to be retrieved.
	 * \param   planeNo  The number of the ORCA finalraint to be retrieved.
	 * \return  A plane representing the specified ORCA finalraint.
	 * \note    The halfspace to which the normal of the plane points is the region of permissible velocities with respect to the specified ORCA finalraint.
	 */
	public Plane getAgentORCAPlane(int agentNo, int planeNo){
		return agents_.get(agentNo).orcaPlanes_.get(planeNo);
	}

	/**
	 * \brief   Returns the three-dimensional position of a specified agent.
	 * \param   agentNo  The number of the agent whose three-dimensional position is to be retrieved.
	 * \return  The present three-dimensional position of the (center of the) agent.
	 */
	public Double3D getAgentPosition(int agentNo){
		return agents_.get(agentNo).position_;
	}
	/**
	 * \brief   Returns the three-dimensional preferred velocity of a specified agent.
	 * \param   agentNo  The number of the agent whose three-dimensional preferred velocity is to be retrieved.
	 * \return  The present three-dimensional preferred velocity of the agent.
	 */
	public Double3D getAgentPrefVelocity(int agentNo){
		return agents_.get(agentNo).prefVelocity_;
	}
	/**
	 * \brief   Returns the radius of a specified agent.
	 * \param   agentNo  The number of the agent whose radius is to be retrieved.
	 * \return  The present radius of the agent.
	 */
	public float getAgentRadius(int agentNo){
		return agents_.get(agentNo).radius_;
	}

	/**
	 * \brief   Returns the time horizon of a specified agent.
	 * \param   agentNo  The number of the agent whose time horizon is to be retrieved.
	 * \return  The present time horizon of the agent.
	 */
	public float getAgentTimeHorizon(int agentNo){
		return agents_.get(agentNo).timeHorizon_;
	}
	/**
	 * \brief   Returns the three-dimensional linear velocity of a specified agent.
	 * \param   agentNo  The number of the agent whose three-dimensional linear velocity is to be retrieved.
	 * \return  The present three-dimensional linear velocity of the agent.
	 */
	public final Double3D getAgentVelocity(int agentNo){
		return agents_.get(agentNo).velocity_;
	}
	/**
	 * \brief   Returns the global time of the simulation.
	 * \return  The present global time of the simulation (zero initially).
	 */
	public float getGlobalTime(){
		return globalTime_;
	}

	/**
	 * \brief   Returns the count of agents in the simulation.
	 * \return  The count of agents in the simulation.
	 */
	public int getNumAgents(){
		return agents_.size();
	}

	/**
	 * \brief   Returns the time step of the simulation.
	 * \return  The present time step of the simulation.
	 */
	public float getTimeStep(){
		return timeStep_;
	}

	/**
	 * \brief   Sets the maximum neighbor count of a specified agent.
	 * \param   agentNo       The number of the agent whose maximum neighbor count is to be modified.
	 * \param   maxNeighbors  The replacement maximum neighbor count.
	 */
	public void setAgentMaxNeighbors(int agentNo, int maxNeighbors){
		agents_.get(agentNo).maxNeighbors_ = maxNeighbors;
	}

	/**
	 * \brief   Sets the maximum speed of a specified agent.
	 * \param   agentNo   The number of the agent whose maximum speed is to be modified.
	 * \param   maxSpeed  The replacement maximum speed. Must be non-negative.
	 */
	public void setAgentMaxSpeed(int agentNo, float maxSpeed){
		agents_.get(agentNo).maxSpeed_ = maxSpeed;
	}

	/**
	 * \brief   Sets the maximum neighbor distance of a specified agent.
	 * \param   agentNo       The number of the agent whose maximum neighbor distance is to be modified.
	 * \param   neighborDist  The replacement maximum neighbor distance. Must be non-negative.
	 */
	public void setAgentNeighborDist(int agentNo, float neighborDist){
		agents_.get(agentNo).neighborDist_ = neighborDist;
	}

	/**
	 * \brief   Sets the three-dimensional position of a specified agent.
	 * \param   agentNo   The number of the agent whose three-dimensional position is to be modified.
	 * \param   position  The replacement of the three-dimensional position.
	 */
	public void setAgentPosition(int agentNo, final Double3D position){
		agents_.get(agentNo).position_ = position;
	}

	/**
	 * \brief   Sets the three-dimensional preferred velocity of a specified agent.
	 * \param   agentNo       The number of the agent whose three-dimensional preferred velocity is to be modified.
	 * \param   prefVelocity  The replacement of the three-dimensional preferred velocity.
	 */
	public void setAgentPrefVelocity(int agentNo, final Double3D prefVelocity){
		agents_.get(agentNo).prefVelocity_ = prefVelocity;
	}

	/**
	 * \brief   Sets the radius of a specified agent.
	 * \param   agentNo  The number of the agent whose radius is to be modified.
	 * \param   radius   The replacement radius. Must be non-negative.
	 */
	public void setAgentRadius(int agentNo, float radius){
		agents_.get(agentNo).radius_ = radius;
	}

	/**
	 * \brief   Sets the time horizon of a specified agent with respect to other agents.
	 * \param   agentNo      The number of the agent whose time horizon is to be modified.
	 * \param   timeHorizon  The replacement time horizon with respect to other agents. Must be positive.
	 */
	public void setAgentTimeHorizon(int agentNo, float timeHorizon){
		agents_.get(agentNo).timeHorizon_ = timeHorizon;
	}

	/**
	 * \brief   Sets the three-dimensional linear velocity of a specified agent.
	 * \param   agentNo   The number of the agent whose three-dimensional linear velocity is to be modified.
	 * \param   velocity  The replacement three-dimensional linear velocity.
	 */
	public void setAgentVelocity(int agentNo, final Double3D velocity){
		agents_.get(agentNo).velocity_ = velocity;
	}

	/**
	 * \brief   Sets the time step of the simulation.
	 * \param   timeStep  The time step of the simulation. Must be positive.
	 */
	public void setTimeStep(float timeStep){
		timeStep_ = timeStep;
	}	

}



/**
 * \brief   Defines a plane.
 */
class Plane {
	/**
	 * \brief   A point on the plane.
	 */
	public Double3D point;

	/**
	 * \brief   The normal to the plane.
	 */
	public Double3D normal;
};