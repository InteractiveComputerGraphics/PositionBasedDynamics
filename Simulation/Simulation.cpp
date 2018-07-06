#include "Simulation.h"
#include "TimeManager.h"
#include "Utils/Timing.h"
#include "TimeStep.h"
#include "TimeStepController.h"

using namespace PBD;
using namespace std;
using namespace GenParam;

Simulation* Simulation::current = nullptr;
int Simulation::GRAVITATION = -1;
int Simulation::SIMULATION_METHOD = -1;
int Simulation::ENUM_SIMULATION_PBD = -1;
int Simulation::ENUM_SIMULATION_XPBD = -1;
int Simulation::ENUM_SIMULATION_IBDS = -1;

Simulation::Simulation () 
{
	m_gravitation = Vector3r(0.0, -9.81, 0.0);

	m_timeStep = nullptr;
	m_simulationMethod = SimulationMethods::NumSimulationMethods;
	m_simulationMethodChanged = NULL;
}

Simulation::~Simulation () 
{
	delete m_timeStep;
	delete TimeManager::getCurrent();

	current = nullptr;
}

Simulation* Simulation::getCurrent ()
{
	if (current == nullptr)
	{
		current = new Simulation ();
		current->init();
	}
	return current;
}

void Simulation::setCurrent (Simulation* tm)
{
	current = tm;
}

bool Simulation::hasCurrent()
{
	return (current != nullptr);
}

void Simulation::init()
{
	initParameters();
	setSimulationMethod(static_cast<int>(SimulationMethods::PBD));
}

void Simulation::initParameters()
{
	ParameterObject::initParameters();

 	GRAVITATION = createVectorParameter("gravitation", "Gravitation", 3u, m_gravitation.data());
 	setGroup(GRAVITATION, "Simulation");
 	setDescription(GRAVITATION, "Vector to define the gravitational acceleration.");

	ParameterBase::GetFunc<int> getSimulationFct = std::bind(&Simulation::getSimulationMethod, this);
	ParameterBase::SetFunc<int> setSimulationFct = std::bind(&Simulation::setSimulationMethod, this, std::placeholders::_1);
	SIMULATION_METHOD = createEnumParameter("simulationMethod", "Simulation method", getSimulationFct, setSimulationFct);
	setGroup(SIMULATION_METHOD, "Simulation");
	setDescription(SIMULATION_METHOD, "Simulation method.");
	EnumParameter* enumParam = static_cast<EnumParameter*>(getParameter(SIMULATION_METHOD));
	enumParam->addEnumValue("Position-Based Dynamics (PBD)", ENUM_SIMULATION_PBD);
	enumParam->addEnumValue("eXtended Position-Based Dynamics (XPBD)", ENUM_SIMULATION_XPBD);
	enumParam->addEnumValue("Impulse-Based Dynamic Simulation (IBDS)", ENUM_SIMULATION_IBDS);
}

void Simulation::reset()
{
	m_model->reset();
	if (m_timeStep)
		m_timeStep->reset();

	TimeManager::getCurrent()->setTime(static_cast<Real>(0.0));
	TimeManager::getCurrent()->setTimeStepSize(static_cast<Real>(0.005));
}

void Simulation::setSimulationMethod(const int val)
{
	SimulationMethods method = static_cast<SimulationMethods>(val);
	if ((method < SimulationMethods::PBD) || (method >= SimulationMethods::NumSimulationMethods))
		method = SimulationMethods::PBD;

	if (method == m_simulationMethod)
		return;

	delete m_timeStep;
	m_timeStep = nullptr;

	m_simulationMethod = method;

	if (method == SimulationMethods::PBD)
	{
		m_timeStep = new TimeStepController();
		m_timeStep->init();
		TimeManager::getCurrent()->setTimeStepSize(static_cast<Real>(0.005));
	}
	else if (method == SimulationMethods::XPBD)
	{
		LOG_INFO << "XPBD not implemented yet.";
	}
	else if (method == SimulationMethods::IBDS)
	{
		LOG_INFO << "IBDS not implemented yet.";
	}	

	if (m_simulationMethodChanged != nullptr)
		m_simulationMethodChanged();
}

void Simulation::setSimulationMethodChangedCallback(std::function<void()> const& callBackFct)
{
	m_simulationMethodChanged = callBackFct;
}

