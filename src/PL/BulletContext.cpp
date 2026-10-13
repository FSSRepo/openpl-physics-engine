#include "BulletContext.h"
#include "pl10.h"

RBQueue::RBQueue():
mass(btScalar(0)),shape(NULL),motion(NULL) {
	inertia = btVector3(btScalar(0),btScalar(0),btScalar(0));
}

btRigidBody* RBQueue::make(DebugLogger* log) {
	if(shape == NULL) {
		return NULL;
	} else {
		if(!motion) {
			if(log) {
				log->warning("[motion state] is null, use instead [procreedTransform:PL_TRANSFORM]");
			}
		}
		btRigidBody::btRigidBodyConstructionInfo info(mass,motion,shape,inertia);
		btRigidBody* body = new btRigidBody(info);
		return body;
	}
}

RayTestResult::RayTestResult():
bodyHitted(-1),hitPoint(btVector3(0,0,0)),hitNormal(btVector3(0,0,0))
{}

ContactCBInfo::ContactCBInfo():
user_ptr(0),col_filter(0),col_flag(1)
{}

CSQueue::CSQueue():
tmp1(0.0f),tmp2(0.0f) {
	tmp3.setValue(0,0,0);
	tmp4.setValue(0,0,0);
	optimizeConvexHull = false;
	vertex_buffer = NULL;
	index_buffer = NULL;
	index_buffer_size = 0;
	vertex_buffer_size = 0;
}
ConstraintQueue::ConstraintQueue(int type):
type_x(type) {
	transA.setIdentity();
	transB.setIdentity();
	tempA.setValue(0,0,0);
	tempB.setValue(0,0,0);
	tempC.setValue(0,0,0);
	tempD.setValue(0,0,0);
	rbA = 0;
	rbB = 0;
	useRefA = false;
	ratio = btScalar(1);
}

btTypedConstraint* ConstraintQueue::make() {
	if(type_x == PL_POINT2PONT_CONSTRAINT) {
		if(rbB == NULL) {
			return new btPoint2PointConstraint(*rbA,tempA);
		}
		return new btPoint2PointConstraint(*rbA,*rbB,tempA,tempB);
	} else if(type_x == PL_SLIDER_CONSTRAINT) {
		if(rbA == NULL) {
			return new btSliderConstraint(*rbB,transB,useRefA);
		}
		return new btSliderConstraint(*rbA,*rbB,transA,transB,useRefA);
	} else if(type_x == PL_HINGE_CONSTRAINT) {
		if(rbB == NULL) {
			if(tempC != NULL) {
				return new btHingeConstraint(*rbA,tempA,tempC,useRefA);
			} else{
				return new btHingeConstraint(*rbA,transA,useRefA);
			}
		} else if(tempA == NULL && tempB == NULL) {
			return new btHingeConstraint(*rbA,*rbB,transA,transB,useRefA);
		}
		return new btHingeConstraint(*rbA,*rbB,tempA,tempB,tempC,tempD,useRefA);
	} else if(type_x == PL_HINGE2_CONSTRAINT) {
		return new btHinge2Constraint(*rbA,*rbB,tempA,tempB,tempC);
	} else if(type_x == PL_G6DOF_CONSTRAINT) {
		return new btGeneric6DofConstraint(*rbA,*rbB,transA,transB,useRefA);
	} else if(type_x == PL_G6DOF_SPRING_CONSTRAINT) {
		if(rbA == NULL) {
			return new btGeneric6DofSpringConstraint(*rbB,transB,useRefA);
		}
		return new btGeneric6DofSpringConstraint(*rbA,*rbB,transA,transB,useRefA);
	} else if(type_x == PL_G6DOF_SPRING2_CONSTRAINT) {
		if(rbA == NULL) {
			return new btGeneric6DofSpring2Constraint(*rbB,transB);
		}
		return new btGeneric6DofSpring2Constraint(*rbA,*rbB,transA,transB);
	} else if(type_x == PL_CONE_TWIST_CONSTRAINT) {
		return new btConeTwistConstraint(*rbA,*rbB,transA,transB);
	} else if(type_x == PL_FIXED_CONSTRAINT) {
		if(rbA == NULL || rbB == NULL) {
			return NULL;
		}
		return new btFixedConstraint(*rbA,*rbB,transA,transB);
	} else if(type_x == PL_GEAR_CONSTRAINT) {
		if(rbA == NULL || rbB == NULL) {
			return NULL;
		}
		return new btGearConstraint(*rbA,*rbB,tempC,tempD,ratio);
	} else if(type_x == PL_UNIVERSAL_CONSTRAINT) {
		if(rbA == NULL || rbB == NULL) {
			return NULL;
		}
		return new btUniversalConstraint(*rbA,*rbB,tempA,tempC,tempD);
	}
	return NULL;
}

VehicleQueue::VehicleQueue() {
	chassis = 0;
}

btRaycastVehicle* VehicleQueue::make(btDefaultVehicleRaycaster* dvrc,btRaycastVehicle::btVehicleTuning tuning) {
	if(!chassis) {
		return NULL;
	}
	return new btRaycastVehicle(tuning, chassis, dvrc);
}

btCollisionShape* CSQueue::make(Plenum type,int* result,DebugLogger* debug) {
	*result = PL_NO_ERROR;
	if(type == PL_BOX_SHAPE) {
		if(tmp3.isZero()) {
			if(debug) {
				debug->warning("plCreate(): the box shape requires half extents value");
			}
			*result = PL_INVALID_VALUE;
			return NULL;
		} else{
			return new btBoxShape(tmp3);
		}
	} else if(type == PL_SPHERE_SHAPE) {
		if(tmp1 <= 0.0f) {
			*result = PL_INVALID_VALUE;
			if(debug) {
				debug->warning("plCreate(): the shape shape requires radius value");
			}
			return NULL;
		} else{
			return new btSphereShape(tmp1);
		}
	} else if(type == PL_CAPSULE_SHAPE) {
		if(tmp1 <= 0.0f || tmp2 <= 0.0f) {
			*result = PL_INVALID_VALUE;
			if(debug) {
				debug->warning("plCreate(): the capsule shape requires radius and height values");
			}
			return NULL;
		} else{
			return new btCapsuleShape(tmp1, tmp2);
		}
	} else if(type == PL_CAPSULE_SHAPE_X) {
		if(tmp1 <= 0.0f || tmp2 <= 0.0f) {
			*result = PL_INVALID_VALUE;
			if(debug) {
				debug->warning("plCreate(): the capsule shape requires radius and height values");
			}
			return NULL;
		} else{
			return new btCapsuleShapeX(tmp1, tmp2);
		}
	} else if(type == PL_CAPSULE_SHAPE_Z) {
		if(tmp1 <= 0.0f || tmp2 <= 0.0f) {
			*result = PL_INVALID_VALUE;
			if(debug) {
				debug->warning("plCreate(): the capsule shape requires radius and height values");
			}
			return NULL;
		} else{
			return new btCapsuleShapeZ(tmp1, tmp2);
		}
	} else if(type == PL_STATIC_PLANE_SHAPE) {
		if(tmp3.isZero()) {
			if(debug) {
				debug->warning("plCreate(): the static plane shape requires normal and constant values");
			}
			*result = PL_INVALID_VALUE;
			return NULL;
		} else{
			return new btStaticPlaneShape(tmp3,tmp1);
		}
	} else if(type == PL_CONE_SHAPE) {
		if(tmp1 <= 0.0f || tmp2 <= 0.0f) {
			if(debug) {
				debug->warning("plCreate(): the cone shape requires radius and height values");
			}
			*result = PL_INVALID_VALUE;
			return NULL;
		} else{
			return new btConeShape(tmp1, tmp2);
		}
	} else if(type == PL_CONE_SHAPE_X) {
		if(tmp1 <= 0.0f || tmp2 <= 0.0f) {
			if(debug) {
				debug->warning("plCreate(): the cone shape requires radius and height values");
			}
			*result = PL_INVALID_VALUE;
			return NULL;
		} else{
			return new btConeShapeX(tmp1, tmp2);
		}
	} else if(type == PL_CONE_SHAPE_Z) {
		if(tmp1 <= 0.0f || tmp2 <= 0.0f) {
			if(debug) {
				debug->warning("plCreate(): the cone shape requires radius and height values");
			}
			*result = PL_INVALID_VALUE;
			return NULL;
		} else{
			return new btConeShapeZ(tmp1, tmp2);
		}
	} else if(type == PL_CYLINDER_SHAPE) {
		if(tmp1 <= 0.0f && tmp2 <= 0.0f) {
			if(debug) {
				debug->warning("plCreate(): the cylinder shape requires radius and height value");
			}
			*result = PL_INVALID_VALUE;
			return NULL;
		} else{
			return new btCylinderShape(btVector3(tmp1, tmp2, tmp1));
		}
	} else if(type == PL_CYLINDER_SHAPE_X) {
		if(tmp1 <= 0.0f && tmp2 <= 0.0f) {
			if(debug) {
				debug->warning("plCreate(): the cylinder shape requires radius and height value");
			}
			*result = PL_INVALID_VALUE;
			return NULL;
		} else{
			return new btCylinderShapeX(btVector3(tmp1, tmp2, tmp1));
		}
	} else if(type == PL_CYLINDER_SHAPE_Z) {
		if(tmp1 <= 0.0f && tmp2 <= 0.0f) {
			if(debug) {
				debug->warning("plCreate(): the cylinder shape requires radius and height value");
			}
			*result = PL_INVALID_VALUE;
			return NULL;
		} else{
			return new btCylinderShapeZ(btVector3(tmp1, tmp2, tmp1));
		}
	} else if(type == PL_COMPOUND_SHAPE) {
		return new btCompoundShape();
	} else if(type == PL_BVH_TRIANGLE_MESH_SHAPE) {
		void* local_vertex_buffer = vertex_buffer;
		void* local_index_buffer = index_buffer;
		if(copy_buffers) {
			local_vertex_buffer = malloc(vertex_buffer_size * sizeof(Plfloat));
			local_index_buffer = malloc(index_buffer_size * sizeof(Plushort));
			memcpy(local_vertex_buffer, vertex_buffer, vertex_buffer_size * sizeof(Plfloat));
			memcpy(local_index_buffer, index_buffer, index_buffer_size * sizeof(Plushort));
		}
		btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();
		btIndexedMesh part;
		part.m_numTriangles = (index_buffer_size / 3);
		part.m_triangleIndexBase = (const unsigned char*)local_index_buffer;
		part.m_triangleIndexStride = 3 * sizeof(Plushort);
		part.m_numVertices = (vertex_buffer_size / 3); // vertexCount
		part.m_vertexBase = (const unsigned char*)local_vertex_buffer;
		part.m_vertexStride = 3 * sizeof(btScalar);
		part.m_indexType = PHY_SHORT;
		meshInterface->addIndexedMesh(part, PHY_SHORT);
		return new btBvhTriangleMeshShape(meshInterface, true, true);
	} else if(type == PL_CONVEX_HULL_SHAPE) {
		btConvexHullShape* shape = new btConvexHullShape(vertex_buffer, vertex_buffer_size / 3, 3 * sizeof(btScalar));
		free(vertex_buffer); // delete because bt convex hull creates a internal copy
		vertex_buffer = NULL;
		if(optimizeConvexHull) {
			shape->optimizeConvexHull();
			optimizeConvexHull = false;
		}
		return shape;
	} else{
		if(debug) {
			debug->warning("plCreate(): invalid shape type");
		}
		*result = PL_INVALID_ENUM;
		return NULL;
	}
}

Character::Character() {
	ghost = new btPairCachingGhostObject();
	ghost->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
}

Character::~Character() {
	if(ghost) {
		delete ghost;
		ghost = NULL;
	}
	if(control) {
		delete control;
		control = NULL;
	}
}

CharacterQueue::CharacterQueue() {
	up.setValue(btScalar(1.0), btScalar(0.0), btScalar(0.0));
	step_height = .75f;
	shape = NULL;
}

Character* CharacterQueue::make(DebugLogger* debug) {
	if(!shape) {
		if(debug) {
			debug->error("CharacterQueue: shape object is null.");
		}
		return NULL;
	}
	Character* character = new Character();
	btPairCachingGhostObject* ghost = character->ghost;
	ghost->setCollisionShape(shape);
	character->control = new btKinematicCharacterController(ghost, shape, step_height, up);
	return character;
}

SoftBodyQueue::SoftBodyQueue(int type) {
	type_x = type;
	vertex_buffer = NULL;
	index_buffer = NULL;
	vertex_buffer_size = 0;
	index_buffer_size = 0;
	copy_buffers = false;
	p0.setValue(btScalar(0),btScalar(0),btScalar(0));
	p1.setValue(btScalar(0),btScalar(0),btScalar(0));
	p2.setValue(btScalar(0),btScalar(0),btScalar(0));
	p3.setValue(btScalar(0),btScalar(0),btScalar(0));
	res_x = 1;
	res_y = 1;
	fixeds = 0;
	radius = btScalar(1);
	total_mass = btScalar(0);
	pressure = btScalar(0);
	stiffness = btScalar(-1);
	damping = btScalar(-1);
	velocity_correction = btScalar(1);
	iterations = 0;
	aero_model = -1;
	randomize_constraints = true;
	anchor_body = NULL;
}

btSoftBody* SoftBodyQueue::make(btSoftBodyWorldInfo* info,DebugLogger* debug) {
	if(!info) {
		return NULL;
	}
	btSoftBody* psb = NULL;
	if(type_x == PL_SOFTBODY_ROPE) {
		psb = btSoftBodyHelpers::CreateRope(*info, p0, p1, res_x < 1 ? 1 : res_x, fixeds);
	} else if(type_x == PL_SOFTBODY_CLOTH) {
		psb = btSoftBodyHelpers::CreatePatch(*info, p0, p1, p2, p3,
			res_x < 1 ? 1 : res_x, res_y < 1 ? 1 : res_y, fixeds, true);
	} else if(type_x == PL_SOFTBODY_ELLIPSOID) {
		if(radius <= 0) {
			if(debug) {
				debug->warning("plCreate(): the ellipsoid soft body requires radius value");
			}
			return NULL;
		}
		psb = btSoftBodyHelpers::CreateEllipsoid(*info, p0,
			btVector3(radius,radius,radius), res_x < 1 ? 1 : res_x);
	} else if(type_x == PL_SOFTBODY_MESH) {
		if(vertex_buffer == NULL || index_buffer == NULL ||
			vertex_buffer_size < 9 || index_buffer_size < 3) {
			if(debug) {
				debug->error("plCreate(): the mesh soft body requires vertex and index buffers");
			}
			return NULL;
		}
		float* local_vertex_buffer = vertex_buffer;
		unsigned short* local_index_buffer = index_buffer;
		if(copy_buffers) {
			local_vertex_buffer = (float*)malloc(vertex_buffer_size * sizeof(float));
			local_index_buffer = (unsigned short*)malloc(index_buffer_size * sizeof(unsigned short));
			memcpy(local_vertex_buffer, vertex_buffer, vertex_buffer_size * sizeof(float));
			memcpy(local_index_buffer, index_buffer, index_buffer_size * sizeof(unsigned short));
		}
		int num_vertices = vertex_buffer_size / 3;
		int num_triangles = index_buffer_size / 3;
		btAlignedObjectArray<btScalar> verts;
		verts.resize(num_vertices * 3);
		for(int i = 0;i < num_vertices;i++) {
			verts[i * 3 + 0] = local_vertex_buffer[i * 3 + 0] + p0.getX();
			verts[i * 3 + 1] = local_vertex_buffer[i * 3 + 1] + p0.getY();
			verts[i * 3 + 2] = local_vertex_buffer[i * 3 + 2] + p0.getZ();
		}
		btAlignedObjectArray<int> tris;
		tris.resize(num_triangles * 3);
		for(int i = 0;i < num_triangles * 3;i++) {
			tris[i] = (int)local_index_buffer[i];
		}
		psb = btSoftBodyHelpers::CreateFromTriMesh(*info, &verts[0], &tris[0],
			num_triangles, randomize_constraints);
		if(copy_buffers) {
			free(local_vertex_buffer);
			free(local_index_buffer);
		}
	} else {
		if(debug) {
			debug->warning("plCreate(): invalid soft body type");
		}
		return NULL;
	}
	if(psb == NULL) {
		return NULL;
	}
	if(stiffness >= 0 && psb->m_materials.size() > 0) {
		psb->m_materials[0]->m_kLST = stiffness;
		psb->m_materials[0]->m_kAST = stiffness;
		psb->m_materials[0]->m_kVST = stiffness;
	}
	if(damping >= 0) {
		psb->m_cfg.kDP = damping;
	}
	psb->m_cfg.kPR = pressure;
	psb->m_cfg.kVCF = velocity_correction;
	if(aero_model >= 0) {
		psb->m_cfg.aeromodel = (btSoftBody::eAeroModel::_)aero_model;
	}
	if(iterations > 0) {
		psb->m_cfg.piterations = iterations;
		psb->m_cfg.viterations = iterations;
	}
	if(total_mass > 0) {
		psb->setTotalMass(total_mass, true);
	}
	if(anchor_body != NULL) {
		if(anchor_nodes.empty()) {
			psb->appendAnchor(0, anchor_body);
		} else {
			for(size_t i = 0;i < anchor_nodes.size();i++) {
				int node = anchor_nodes[i];
				if(node >= 0 && node < psb->m_nodes.size()) {
					psb->appendAnchor(node, anchor_body);
				}
			}
		}
	}
	return psb;
}

BulletContext::BulletContext(int numThreads):
world(NULL),error_casting(PL_NO_ERROR),
body_queue(NULL),shape_queue(NULL),char_queue(NULL),
veh_queue(NULL),const_queue(NULL),soft_queue(NULL)
{
	if(numThreads > (int)BT_MAX_THREAD_COUNT) {
		numThreads = (int)BT_MAX_THREAD_COUNT;
	}
	solver = NULL;
	solver_pool = NULL;
	solver_mt = NULL;
	task_scheduler = NULL;
	world_mt = NULL;
	is_multithreaded = false;
	num_threads = 1;

	config = new btSoftBodyRigidBodyCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(config);
	broadphase = new btDbvtBroadphase();

	if(numThreads > 1) {
		// multithreaded world: uses Bullet's internal task scheduler
		btITaskScheduler* ts = btCreateDefaultTaskScheduler();
		if(ts != NULL) {
			ts->setNumThreads(numThreads);
			btSetTaskScheduler(ts);
			task_scheduler = ts;
			solver_pool = new btConstraintSolverPoolMt(numThreads);
			solver_mt = new btSequentialImpulseConstraintSolverMt();
			world_mt = new btDiscreteDynamicsWorldMt(dispatcher, broadphase, solver_pool, solver_mt, config);
			is_multithreaded = true;
			num_threads = numThreads;
		}
	}

	if(is_multithreaded) {
		// default gravity 9.81 m/s2 in Y axis
		world_mt->setGravity(btVector3(0.0f, -9.81f, 0.0f));
	} else {
		// single-threaded soft-rigid world (soft body capable)
		solver = new btSequentialImpulseConstraintSolver();
		world = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, config);
		if(world != NULL) {
			// default gravity 9.81 m/s2 in Y axis
			world->setGravity(btVector3(0.0f, -9.81f, 0.0f));
			world->getWorldInfo().m_gravity = world->getGravity();
		}
	}
	bodies = new vector<btRigidBody*>();
	shapes = new vector<btCollisionShape*>();
	raytest = new vector<RayTestResult*>();
	constraints = new vector<btTypedConstraint*>();
	vehicles = new vector<btRaycastVehicle*>();
	characters = new vector<Character*>();
	soft_bodies = new vector<btSoftBody*>();
	cur_body = 0;
	cur_shape = 0;
	cur_const = 0;
	cur_char = 0;
	cur_veh = 0;
	cur_soft = 0;
	flag_ray_test = 0;
	vehraycaster = NULL;
	debug_mode = false;
	has_global_error = false;
	contact_testing = false;
	axis_sweep_use = false;
	is_character_context = false;
	is_softbody_context = false;
	debug = NULL;
	shapeTrans.setIdentity();
}

BulletContext::~BulletContext() {
	clearRayTest();
	for(int i = 1;i <= bodies->size();i++) {
		deleteRigidBody(i);
	}
	if(debug != NULL) {
		delete debug;
		debug = NULL;
	}
	if(getWorld() != NULL && getWorld()->getNumConstraints() > 0) {
		for(int i = 0;i < getWorld()->getNumConstraints();i++) {
			btTypedConstraint* c = getWorld()->getConstraint(i);
			getWorld()->removeConstraint(c);
			delete c;
		}
	}
	if(vehicles->size() > 0) {
		for(int i = 1;i <= vehicles->size();i++) {
			deleteVehicle(i);
		}
	}
	if(characters->size() > 0) {
		for(int i = 1;i <= characters->size();i++) {
			deleteCharacter(i);
		}
	}
	if(soft_bodies->size() > 0) {
		for(int i = 1;i <= soft_bodies->size();i++) {
			deleteSoftBody(i);
		}
	}
	bodies = NULL;
	raytest = NULL;
	if(is_multithreaded) {
		delete world_mt;
		world_mt = NULL;
		delete solver_pool;
		solver_pool = NULL;
		delete solver_mt;
		solver_mt = NULL;
		if(task_scheduler != NULL) {
			btSetTaskScheduler(btGetSequentialTaskScheduler());
			delete task_scheduler;
			task_scheduler = NULL;
		}
	} else {
		delete world;
		world = NULL;
		delete solver;
		solver = NULL;
	}
	delete dispatcher;
	delete broadphase;
	delete vehraycaster;
	vehraycaster = 0;
}

void BulletContext::addRigidBody(btRigidBody* body) {
	bodies->push_back(body);
}

void BulletContext::addCollisionShape(btCollisionShape* shape) {
	shapes->push_back(shape);
}

void BulletContext::addConstraint(btTypedConstraint* constraint) {
	constraints->push_back(constraint);
}

void BulletContext::addVehicle(btRaycastVehicle* veh) {
	vehicles->push_back(veh);
}

void BulletContext::addCharacter(Character* character) {
	characters->push_back(character);
}
	
void BulletContext::deleteCharacter(Pluint indx) {
	Character* ch = characters->at(indx - 1);
	if(ch != NULL) {
		characters->at(indx - 1) = 0;
		getWorld()->removeAction(ch->control);
		getWorld()->removeCollisionObject(ch->ghost);
		delete ch;
		ch =  NULL;
		return;
	}
	if(debug_mode) {
		debug->warning("plDeleteCharacter(): this character is deleted or not exist.");
	}
	error_casting = PL_INVALID_NAME;
}

void BulletContext::addSoftBody(btSoftBody* soft) {
	soft_bodies->push_back(soft);
}

void BulletContext::deleteSoftBody(Pluint indx) {
	btSoftBody* sb = soft_bodies->at(indx - 1);
	if(sb != NULL) {
		soft_bodies->at(indx - 1) = 0;
		if(world != NULL && world->getSoftBodyArray().findLinearSearch(sb) >= 0) {
			world->removeSoftBody(sb);
		}
		delete sb;
		sb = NULL;
		return;
	}
	if(debug_mode) {
		debug->warning("plDeleteSoftBody(): this soft body is deleted or not exist.");
	}
	error_casting = PL_INVALID_NAME;
}

void BulletContext::deleteRigidBody(Pluint indx) {
	btRigidBody* rb = bodies->at(indx - 1);
	if(rb != NULL) {
		bodies->at(indx - 1) = 0;
		if(rb->isInWorld()) {
			getWorld()->removeRigidBody(rb);
		}
		delete rb->getMotionState();
		Pluint tmp = findCollisionShape(rb->getCollisionShape());
		btCollisionShape* shape = shapes->at(tmp);
		shapes->at(tmp) = 0;
		delete shape;
		delete rb;
		return;
	}
	if(debug_mode) {
		debug->warning("plDeleteBody(): this body is deleted or not exist.");
	}
	error_casting = PL_INVALID_NAME;
}

void BulletContext::deleteVehicle(Pluint indx) {
	btRaycastVehicle* veh = vehicles->at(indx - 1);
	if(veh) {
		delete veh;
		veh = NULL;
		return;
	}
	if(debug_mode) {
		debug->warning("plDeleteVehicle(): this vehicle is deleted or not exist.");
	}
	error_casting = PL_INVALID_NAME;
}

void BulletContext::deleteConstraint(Pluint indx) {
	btTypedConstraint* tc = constraints->at(indx - 1);
	if(tc) {
		delete tc;
		tc = NULL;
		return;
	}
	if(debug_mode) {
		debug->warning("plDeleteConstraint(): this body is deleted or not exist.");
	}
	error_casting = PL_INVALID_NAME;
}

void BulletContext::rayTest(const btVector3& from,const btVector3& to, bool all) {
	clearRayTest();
	if(!all) {
		btCollisionWorld::ClosestRayResultCallback ray_results(from, to);
		ray_results.m_flags = flag_ray_test;
    	getWorld()->rayTest(from, to, ray_results);
    	if (ray_results.hasHit())
    	{
       	 	btRigidBody* body = (btRigidBody*)btRigidBody::upcast(ray_results.m_collisionObject);
        	if (body) {
				RayTestResult* test = new RayTestResult();
				test->bodyHitted = findRigidBody(body);
				test->hitPoint = ray_results.m_hitPointWorld;
				test->hitNormal = ray_results.m_hitNormalWorld;
				raytest->push_back(test);
       		}
    	}
	} else{
		btCollisionWorld::AllHitsRayResultCallback allResults(from,to);
		allResults.m_flags = flag_ray_test;
		getWorld()->rayTest(from,to,allResults);
		for (int i = 0;i < allResults.m_hitFractions.size(); i++)
           {
               btRigidBody* body = (btRigidBody*)btRigidBody::upcast(allResults.m_collisionObjects[i]);
        		if (body && raytest->size() < PL_MAX_RAY_RESULTS) {
					RayTestResult* test = new RayTestResult();
					test->bodyHitted = findRigidBody(body);
					test->hitPoint = allResults.m_hitPointWorld[i];
					test->hitNormal = allResults.m_hitNormalWorld[i];
					raytest->push_back(test);
       			}
           }
	}
	flag_ray_test = 0;
}

void BulletContext::clearRayTest() {
	if(raytest->size() > 0) {
		raytest->clear();
	}
}

Plint BulletContext::getLastError() {
	Plint error;
	error = error_casting;
	error_casting = PL_NO_ERROR;
	return error;
}
