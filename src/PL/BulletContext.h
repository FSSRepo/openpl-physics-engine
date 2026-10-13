#ifndef _BULLETCONTEXT_H_
#define _BULLETCONTEXT_H_

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"
#include "LinearMath/btThreads.h"
#include "debug.h"
#include <vector>

typedef unsigned int Pluint;
typedef unsigned char Plubyte;
typedef float Plfloat;
typedef unsigned short Plushort;
typedef int Plint;


using std::vector;

class RBQueue {
	public:
	RBQueue();
	btRigidBody* make(DebugLogger* log);
	btCollisionShape* shape;
	btMotionState* motion;
	btScalar mass;
	btVector3 inertia;
};

class RayTestResult {
	public:
	RayTestResult();
	Pluint bodyHitted;
	btVector3 hitPoint;
	btVector3 hitNormal;
};

class ContactCBInfo {
	public:
	ContactCBInfo();
	int user_ptr;
	int col_filter;
	int col_flag;
};

class CSQueue {
	public:
	CSQueue();
	btCollisionShape* make(int type, int* result, DebugLogger* debug);
	btScalar tmp1;
	btScalar tmp2;
	btVector3 tmp3;
	btVector3 tmp4;

	unsigned short* index_buffer;
	float* vertex_buffer;

	/*
		This could cause crashes that
		it is immediately deleted
	*/

	int index_buffer_size;
	int vertex_buffer_size;
	bool optimizeConvexHull;
	bool copy_buffers = false;
};

class ConstraintQueue {
	public:
	ConstraintQueue(int type);
	btTypedConstraint* make();
	int type_x;
	btRigidBody* rbA;
	btRigidBody* rbB;
	btTransform transA;
	btTransform transB;
	btVector3 tempA;
	btVector3 tempB;
	btVector3 tempC;
	btVector3 tempD;
	bool useRefA;
	btScalar ratio;
};

class VehicleQueue {
	public:
	VehicleQueue();
	btRigidBody* chassis;
	btRaycastVehicle* make(btDefaultVehicleRaycaster* dvrc,btRaycastVehicle::btVehicleTuning tuning);
};

class Character {
	public:
	Character();
	~Character();
	btKinematicCharacterController* control;
	btPairCachingGhostObject* ghost;
};

class CharacterQueue {
	public:
	CharacterQueue();
	Character* make(DebugLogger* debug);
	btConvexShape* shape;
	btScalar step_height;
	btVector3 up;
};

class SoftBodyQueue {
	public:
	SoftBodyQueue(int type);
	btSoftBody* make(btSoftBodyWorldInfo* info,DebugLogger* debug);
	int type_x;
	float* vertex_buffer;
	unsigned short* index_buffer;
	int vertex_buffer_size;
	int index_buffer_size;
	bool copy_buffers;
	btVector3 p0;
	btVector3 p1;
	btVector3 p2;
	btVector3 p3;
	int res_x;
	int res_y;
	int fixeds;
	btScalar radius;
	btScalar total_mass;
	btScalar pressure;
	btScalar stiffness;
	btScalar damping;
	btScalar velocity_correction;
	int iterations;
	int aero_model;
	bool randomize_constraints;
	btRigidBody* anchor_body;
	vector<int> anchor_nodes;
};

class BulletContext {
	public:
	// temporaly variables delete when not use it
	RBQueue *body_queue;
	CSQueue *shape_queue;
	
	/* queue of constraint contructor */
	ConstraintQueue* const_queue;
	VehicleQueue* veh_queue;
	CharacterQueue* char_queue;
	SoftBodyQueue* soft_queue;
	
	/* for contact, ray testing and debugging*/
	int flag_ray_test;
	bool debug_mode;
	bool has_global_error;
	DebugLogger* debug;
	bool contact_testing;
	
	// for integrate the engine
	Pluint cur_body;
	Pluint cur_shape;
	Pluint cur_const;
	Pluint cur_veh;
	Pluint cur_char;
	Pluint cur_soft;
	btTransform shapeTrans;
	btRaycastVehicle::btVehicleTuning tuning;
	
	BulletContext(int numThreads = 1);
	virtual ~BulletContext();
	
	btDiscreteDynamicsWorld* getWorld() {
		return is_multithreaded ? static_cast<btDiscreteDynamicsWorld*>(world_mt)
		                        : static_cast<btDiscreteDynamicsWorld*>(world);
	}
	
	btSoftRigidDynamicsWorld* getSoftWorld() {
		return world;
	}
	
	bool isMultithreaded() {
		return is_multithreaded;
	}
	
	Plint getNumThreads() {
		return num_threads;
	}
	
	btDefaultVehicleRaycaster* getVehRayCaster() {
		return vehraycaster;
	}
	void addRigidBody(btRigidBody* body);
	
	void deleteRigidBody(Pluint indx);
	
	void addCollisionShape(btCollisionShape* shape);
	
	void addConstraint(btTypedConstraint* constraint);
	
	void deleteConstraint(Pluint indx);
	
	void addVehicle(btRaycastVehicle* veh);
	
	void deleteVehicle(Pluint indx);
	
	void addCharacter(Character* character);
	
	void deleteCharacter(Pluint indx);
	
	void addSoftBody(btSoftBody* soft);
	
	void deleteSoftBody(Pluint indx);
	
	btSoftBody* getSoftBody(Pluint indx) {
		if((indx - 1) >= soft_bodies->size() || indx <= 0) {
			return NULL;
		}
		return soft_bodies->at(indx - 1);
	}
	
	btSoftBodyWorldInfo* getSoftBodyWorldInfo() {
		if(world == NULL) {
			return NULL;
		}
		return &world->getWorldInfo();
	}
	
	void setGravity(const btVector3& gravity) {
		getWorld()->setGravity(gravity);
		if(world != NULL) {
			world->getWorldInfo().m_gravity = gravity;
		}
	}
	
	void initSoftBodyContext() {
		if(world == NULL) {
			if(debug_mode) {
				debug->warning("plDynamicWorldi(): soft body context is unavailable in a multithreaded world.");
			}
			return;
		}
		is_softbody_context = true;
		getSoftBodyWorldInfo()->m_gravity = world->getGravity();
		if(debug_mode) {
			debug->info("plDynamicWorldi(): soft body context is enabled.");
		}
	}
	
	bool isSoftBodyContext() {
		return is_softbody_context;
	}
	
	void initVehicleRayCaster() {
		vehraycaster = new btDefaultVehicleRaycaster(getWorld());
	}
	
	btCollisionShape* getCollisionShape(Pluint indx) {
		if((indx - 1) >= shapes->size() || indx <= 0) {
			return NULL;
		}
		if(shapes->at(indx - 1) == NULL) {
			return NULL;
		} else{
			return shapes->at(indx - 1);
		}
	}
	
	btRigidBody* getRigidBody(Pluint indx) {
		if((indx - 1) >= bodies->size() || indx <= 0) {
			return NULL;
		}
		if(bodies->at(indx - 1) == NULL) {
			return NULL;
		} else{
			return bodies->at(indx - 1);
		}
	}
	
	btTypedConstraint* getConstraint(Pluint indx) {
		if((indx - 1) >= constraints->size() || indx <= 0) {
			return NULL;
		}
		if(constraints->at(indx - 1) == NULL) {
			return NULL;
		} else{
			return constraints->at(indx - 1);
		}
	}
	
	btRaycastVehicle* getVehicle(Pluint indx) {
		if((indx - 1) >= vehicles->size() || indx <= 0) {
			return NULL;
		}
		if(vehicles->at(indx - 1) == NULL) {
			return NULL;
		} else{
			return vehicles->at(indx - 1);
		}
	}
	
	Character* getCharacter(Pluint indx) {
		if((indx - 1) >= characters->size() || indx <= 0) {
			return NULL;
		}
		if(characters->at(indx - 1) == NULL) {
			return NULL;
		} else{
			return characters->at(indx - 1);
		}
	}
	
	void rayTest(const btVector3& from,const btVector3& to,bool all);
	
	Pluint queueConstraint(int type) {
		const_queue = new ConstraintQueue(type);
		return constraints->size() + 1;
	}
	
	Pluint queueRigidBody() {
		body_queue = new RBQueue();
		return bodies->size() + 1;
	}
	
	Pluint queueShape() {
		shape_queue = new CSQueue();
		return shapes->size() + 1;
	}
	
	Pluint queueVehicle() {
		veh_queue = new VehicleQueue();
		return vehicles->size() + 1;
	}
	
	Pluint queueCharacter() {
		char_queue = new CharacterQueue();
		return characters->size() + 1;
	}
	
	Pluint queueSoftBody(int type) {
		soft_queue = new SoftBodyQueue(type);
		return soft_bodies->size() + 1;
	}
	
	void setBroadphase(btBroadphaseInterface* newbroadphase, bool axis_sweep) {
		getWorld()->setBroadphase(newbroadphase);
		broadphase = NULL;
		broadphase = newbroadphase;
		axis_sweep_use = axis_sweep;
	}
	
	void setError(const int error) {
		error_casting = error;
	}
	
	Plint getLastError();
	
	bool isAxisSweep() {
		return axis_sweep_use;
	}

	bool ischaracterContext() {
		return is_character_context;
	}
	
	bool isTheContextCreated() {
		return 
			(getWorld() != NULL) && 
			(shapes != NULL) && 
			(bodies != NULL);
	}
	
	RayTestResult* getRayTest(int indx) {
		return raytest->at(indx);
	}
	
	Plint getNumRayTest() {
		return raytest->size();
	}
	
	bool hasRayTest() {
		return raytest->size() > 0;
	}
	
	void clearRayTest();
	
	bool col_filter(ContactCBInfo* c0,ContactCBInfo* c1) {
		return (c0->col_filter & c1->col_flag) == c1->col_flag;
	}
	
	bool findRigidBodyByUserPointer(int user_ptr) {
		for(Pluint i = 0; i < bodies->size(); i++) {
			void* ptr = bodies->at(i)->getUserPointer();
			if(ptr == NULL)
				continue;
			if(ptr == reinterpret_cast<void*>(user_ptr)) {
				cur_body = i + 1;
				return true;
			}
		}
		cur_body = 0;
		return false;
	}
	
	void initCharacterContext() {
		if(is_character_context) {
			if(debug_mode) {
				debug->warning("plDynamicWorldi(): character context already has been enabled.");
			}
			return;
		}
		is_character_context = true;
		getWorld()->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
		if(debug_mode) {
			debug->info("plDynamicWorldi(): character context is enabled.");
		}
	}
	
	private:
	Pluint findCollisionShape(btCollisionShape* shape) {
		for(Pluint i = 0; i < shapes->size(); i++) {
			if(shapes->at(i) == shape) {
				return i;
			}
		}
		return -1;
	}
	
	Pluint findRigidBody(btRigidBody* body) {
		for(Pluint i = 0; i < bodies->size(); i++) {
			if(bodies->at(i) == body) {
				return i;
			}
		}
		return -1;
	}

	Pluint error_casting;
	bool axis_sweep_use;
	bool is_character_context;
	bool is_softbody_context;

	// physics world temporaly definitions
	btDefaultCollisionConfiguration* config;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* broadphase;
	btSequentialImpulseConstraintSolver* solver;

	// multithreading (used instead of solver/world when is_multithreaded is true)
	btConstraintSolverPoolMt* solver_pool;
	btSequentialImpulseConstraintSolverMt* solver_mt;
	btITaskScheduler* task_scheduler;
	btDiscreteDynamicsWorldMt* world_mt;
	bool is_multithreaded;
	Plint num_threads;

	// context defines
	btDefaultVehicleRaycaster* vehraycaster;
	btSoftRigidDynamicsWorld *world;
	vector<RayTestResult*> *raytest;
	vector<btCollisionShape*> *shapes;
	vector<btRigidBody*> *bodies;
	vector<btTypedConstraint*> *constraints;
	vector<btRaycastVehicle*> *vehicles;
	vector<Character*> *characters;
	vector<btSoftBody*> *soft_bodies;
};

#endif
