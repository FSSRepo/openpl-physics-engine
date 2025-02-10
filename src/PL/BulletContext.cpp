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

BulletContext::BulletContext():
world(NULL),error_casting(PL_NO_ERROR),
body_queue(NULL),shape_queue(NULL),char_queue(NULL),
veh_queue(NULL),const_queue(NULL)
{
	config = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(config);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();
	world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
	if(world != NULL) {
		// default gravity 9.81 m/s2 in Y axis
		world->setGravity(btVector3(0.0f, -9.81f, 0.0f));
	}
	bodies = new vector<btRigidBody*>();
	shapes = new vector<btCollisionShape*>();
	raytest = new vector<RayTestResult*>();
	constraints = new vector<btTypedConstraint*>();
	vehicles = new vector<btRaycastVehicle*>();
	characters = new vector<Character*>();
	cur_body = 0;
	cur_shape = 0;
	cur_const = 0;
	cur_char = 0;
	cur_veh = 0;
	flag_ray_test = 0;
	vehraycaster = NULL;
	debug_mode = false;
	has_global_error = false;
	contact_testing = false;
	axis_sweep_use = false;
	is_character_context = false;
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
	if(world->getNumConstraints() > 0) {
		for(int i = 0;i < world->getNumConstraints();i++) {
			btTypedConstraint* c = world->getConstraint(i);
			world->removeConstraint(c);
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
	bodies = NULL;
	raytest = NULL;
	delete world;
	delete dispatcher;
	delete broadphase;
	delete solver;
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
		world->removeAction(ch->control);
		world->removeCollisionObject(ch->ghost);
		delete ch;
		ch =  NULL;
		return;
	}
	if(debug_mode) {
		debug->warning("plDeleteCharacter(): this character is deleted or not exist.");
	}
	error_casting = PL_INVALID_NAME;
}

void BulletContext::deleteRigidBody(Pluint indx) {
	btRigidBody* rb = bodies->at(indx - 1);
	if(rb != NULL) {
		bodies->at(indx - 1) = 0;
		if(rb->isInWorld()) {
			world->removeRigidBody(rb);
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
    	world->rayTest(from, to, ray_results);
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
		world->rayTest(from,to,allResults);
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
