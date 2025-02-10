#include "pl10.h"
#include "BulletContext.h"

BulletContext* ctx = NULL;
obtContactCallBack contactCB = 0;

inline void REPORT_ERROR(int error) {
	ctx->setError(error);
}

inline bool callbackFunc(btManifoldPoint& cp,const btCollisionObjectWrapper* obj1,int id1,int index1,const btCollisionObjectWrapper* obj2,int id2,int index2) {
	if (!ctx->contact_testing) {
		if (ctx->debug_mode) {
			ctx->debug->error("ContactTestError: return 0");
		}
		return false;
	}
	if (ctx->debug_mode) {
		ctx->debug->info("Collision");
	}
	const btCollisionObject* co0 = obj1->m_collisionObject;
	const btCollisionObject* co1 = obj2->m_collisionObject;
	bool match0 = ctx->col_filter(((ContactCBInfo*)co0->getUserPointer()),((ContactCBInfo*)co1->getUserPointer()));
	bool match1 = ctx->col_filter(((ContactCBInfo*)co1->getUserPointer()),((ContactCBInfo*)co0->getUserPointer()));
	if (contactCB) {
		if (ctx->debug_mode) {
			ctx->debug->info("INTERNAL: contact testing");
		}
		return (!match0 && !match1) ? false : contactCB(((ContactCBInfo*)co0->getUserPointer())->user_ptr,match0,((ContactCBInfo*)co1->getUserPointer())->user_ptr,match1);
	}
	return false;
}

inline int getParamLen(Plenum param) {
	switch(param) {
		case PL_POSITION:
		case PL_LINEAR_FACTOR:
		case PL_ANGULAR_FACTOR:
		case PL_LOCAL_INERTIA:
		case PL_APPLY_CENTRAL_FORCE:
		case PL_APPLY_TORQUE:
		case PL_MOTION_STATE_POSITION:
		case PL_GRAVITY:
			return 3;
		case PL_ORIENTATION:
			return 4;
		case PL_BASIS:
			return 9;
		case PL_TRANSFORM:
		case PL_MOTION_STATE_TRANSFORM:
		case PL_COL_WORLD_TRANSFORM:
		case PL_COL_INTERP_WORLD_TRANSFORM:
			return 16;
		case PL_APPLY_FORCE:
		case PL_APPLY_IMPULSE:
		case PL_RAY_CLOSEST_TEST:
		case PL_RAY_ALL_TEST:
			return 6;
		default:
		if (ctx->debug_mode) {
			ctx->debug->error("method (fv): parameter invalid")->append(" (")->hex(param)->append(")");
		}
			REPORT_ERROR(PL_INVALID_ENUM);
			return -1;
	}
}

inline bool hasContext() {
	return ctx != NULL;
}

/*
	Main Functions of API
*/

PLbool plCreateContext() {
	if (!hasContext()) {
		ctx = new BulletContext();
		if (!ctx) {
			return 0;
		} else {
			if (ctx->isTheContextCreated()) {
				return 1;
			} else {
				ctx = NULL;
				return 0;
			}
		}
	} else {
		if (ctx->debug_mode) {
			ctx->debug->warning("plCreateContext(): context has already been created");
		}
		return 1;
	}
}

void plDestroyContext() {
	if (ctx == NULL) {
		return;
	}
	delete ctx;
	ctx = NULL;
}

Plint plGetError() {
	if (!hasContext()) {
		return PL_INVALID_OPERATION;
	}
	return ctx->getLastError();
}

const char* plGetString(Plenum param) {
	if (!hasContext()) {
		return "";
	}
	switch(param) {
		case PL_VERSION:
			return "Open Physics Library 1.0";
		case PL_VENDOR:
			return "Fast Smart System";
		case PL_VBASED:
			return "Based in: Bullet Physics 2.87 [2014]";
		case PL_DEBUG_INFO:
			if (!ctx->debug_mode || !ctx->debug) {
				ctx->debug_mode = false;
				return "Debug Mode: OFF";
			}
			return ctx->debug->toString(ctx->has_global_error);
		default:
		return "";
	}
}

Plint plGetInteger(Plenum param) {
	if (!hasContext()) {
		return 0;
	}
	switch(param) {
		case PL_VERSION:
			return 001;
		case PL_VBASED:
			return 287;
		default:
		if (ctx->debug_mode) {
			ctx->debug->error("plGetInteger(): parameter not supported.");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
		return 0;
	}
}

void plStepSimulation(Plfloat timeStep, Plint maxSubSteps, Plfloat fixedTimeStep) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	ctx->getWorld()->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
}

/*
	Dynamics World settes and gettes
*/

void plDynamicWorldi(Plenum param, Plint value) {
	if (!hasContext()) {
		return;
	}
	if (param == PL_ADD_RIGID_BODY) {
		btRigidBody* b = ctx->getRigidBody(value);
		if (b == NULL) {
			if (ctx->debug_mode) {
				ctx->debug->error("plDynamicWorldi(): body not exist");
			}
			ctx->has_global_error = true;
			REPORT_ERROR(PL_INVALID_NAME);
			return;
		}
		ctx->getWorld()->addRigidBody(b);
		return;
	} else if (param == PL_REMOVE_RIGID_BODY) {
		btRigidBody* b = ctx->getRigidBody(value);
		if (b == NULL) {
			if (ctx->debug_mode) {
				ctx->debug->error("plDynamicWorldi(): body not exist");
			}
			REPORT_ERROR(PL_INVALID_NAME);
			ctx->has_global_error = true;
			return;
		}
		if (b->isInWorld()) {
			ctx->getWorld()->removeRigidBody(b);
		}
		return;
	} else if (param == PL_ADD_CONSTRAINT) {
		btTypedConstraint* c = ctx->getConstraint(value);
		if (c == NULL) {
			if (ctx->debug_mode) {
				ctx->debug->error("plDynamicWorldi(): constraint not exist");
			}
			REPORT_ERROR(PL_INVALID_NAME);
			ctx->has_global_error = true;
			return;
		}
		ctx->getWorld()->addConstraint(c);
		return;
	} else if (param == PL_REMOVE_CONSTRAINT) {
		btTypedConstraint* c = ctx->getConstraint(value);
		if (c == NULL) {
			if (ctx->debug_mode) {
				ctx->debug->error("plDynamicWorldi(): constraint not exist");
			}
			REPORT_ERROR(PL_INVALID_NAME);
			ctx->has_global_error = true;
			return;
		}
		ctx->getWorld()->removeConstraint(c);
		return;
	} else if (param == PL_INIT_VEHICLE_RAYCASTER) {
		ctx->initVehicleRayCaster();
		if (ctx->debug_mode) {
				ctx->debug->info("plDynamicWorldi(): vehicle ray caster context is initialized.");
		}
		return;
	} else if (param == PL_INIT_CHARACTER) {
		if (!ctx->isAxisSweep()) {
			if (ctx->debug_mode) {
				ctx->debug->warning("plDynamicWorldi(): use AxisSweep3. Otherwise the character context may not work!");
			}
		}
		ctx->initCharacterContext();
		return;
	} else if (param == PL_ADD_CHARACTER) {
		Character* ch = ctx->getCharacter(value);
		if (!ch) {
			REPORT_ERROR(PL_INVALID_NAME);
			if (ctx->debug_mode) {
				ctx->debug->error("plDynamicWorldi(): invalid name");
			}
			return;
		}
		if(!ctx->ischaracterContext()) {
			REPORT_ERROR(PL_INVALID_OPERATION);
			if (ctx->debug_mode) {
				ctx->debug->error("plDynamicWorldi(): character context is disabled");
			}
			return;
		}
		ctx->getWorld()->addAction(ch->control);
		ctx->getWorld()->addCollisionObject(ch->ghost, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
		return;
	} else if (param == PL_ADD_VEHICLE) {
		btRaycastVehicle* v = ctx->getVehicle(value);
		if (v == NULL) {
			if (ctx->debug_mode) {
				ctx->debug->error("plDynamicWorldi(): vehicle not exist");
			}
			REPORT_ERROR(PL_INVALID_NAME);
			ctx->has_global_error = true;
			return;
		}
		ctx->getWorld()->addVehicle(v);
		return;
	}
	switch(param) {
		case PL_CLEAR_RAY_RESULTS:
			ctx->clearRayTest();
			break;
		case PL_USE_DBVT:
			if (ctx->isAxisSweep()) {
				ctx->setBroadphase(new btDbvtBroadphase(),false);
			}
			break;
		case PL_CONTACT_TEST:
			ctx->contact_testing = (value == 1);
			break;
		case PL_USER_POINTER:
			if (ctx->findRigidBodyByUserPointer(value)) {
				if (ctx->debug_mode) {
					ctx->debug->info("plDynamicWorldi(): rigid body founded and binded");
				}
			}
			break;
		case PL_ADD_FLAG_RAY_TEST:
			ctx->flag_ray_test |= value;
			break;
		case PL_DEBUG_MODE:
			ctx->debug_mode = (value == PL_TRUE);
			if (ctx->debug_mode && !ctx->debug) {
				ctx->debug = new DebugLogger();
			} else {
				ctx->debug = NULL;
			}
			break;
		default:
		if (ctx->debug_mode) {
			ctx->debug->error("plDynamicWorldi(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		ctx->has_global_error = true;
		REPORT_ERROR(PL_INVALID_ENUM);
		break;
	}
}

void plDynamicWorld3f(Plenum param, Plfloat v0, Plfloat v1, Plfloat v2) {
	if (!hasContext()) {
		return;
	}
	switch(param) {
		case PL_GRAVITY:
			ctx->getWorld()->setGravity(
				btVector3(v0,v1,v2));
			if (ctx->debug_mode) {
				ctx->debug->info("Setup gravity (")->pf(v0)->append(", ")->pf(v1)->append(", ")->pf(v2)->append(")");
			}
			break;
		case PL_USE_AXIS_SWEEP3:
			/* using a bounding box */
			ctx->setBroadphase(
				new btAxisSweep3(
					btVector3(-v0,-v1,-v2),
					btVector3( v0, v1, v2)), true);
			break;
		default:
		if (ctx->debug_mode) {
			ctx->debug->error("plDynamicWorld3f(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		ctx->has_global_error = true;
		REPORT_ERROR(PL_INVALID_ENUM);
		break;
	}
}

void plDynamicWorldfv(Plenum param, Plfloat* values, Plsizei length) {
	if (!hasContext()) {
		return;
	}
	int numLen = getParamLen(param);
	if (numLen == -1) {
		return;
	}
	if (length == 0 ||  length != numLen) {
		REPORT_ERROR(PL_INVALID_VALUE);
		if (ctx->debug_mode) {
			ctx->debug->warning("plDynamicWorldfv(): invalid length value.");
		}
		return;
	}
	switch(param) {
		case PL_GRAVITY:
			ctx->getWorld()->setGravity(
				btVector3(values[0],values[1],values[2]));
			return;
		case PL_RAY_CLOSEST_TEST:
			ctx->rayTest(
				btVector3(values[0],values[1],values[2]),
				btVector3(values[3],values[4],values[5]), false);
			return;
		case PL_RAY_ALL_TEST:
			ctx->rayTest(
				btVector3(values[0],values[1],values[2]),
				btVector3(values[3],values[4],values[5]), true);
			return;
		case PL_USE_AXIS_SWEEP3:
			// order [min and max]
			ctx->setBroadphase(
				new btAxisSweep3(
					btVector3(values[0],values[1],values[2]),
					btVector3(values[3],values[4],values[5])), true);
			break;
		default:
		if (ctx->debug_mode) {
			ctx->debug->error("plDynamicWorldfv(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		ctx->has_global_error = true;
		REPORT_ERROR(PL_INVALID_ENUM);
		break;
	}
}

void plGetDynamicWorldi(Plenum param, Plint* value) {
	if (!hasContext()) {
		return;
	}
	switch(param) {
		case PL_NUM_RIGID_BODY:
			*value = ctx->getWorld()->getNumCollisionObjects();
			break;
		case PL_RAY_BODY_RESULT:
			if (!ctx->hasRayTest()) {
				*value = -1;
				return;
			}
			*value = ctx->getRayTest(0)->bodyHitted;
			return;
		case PL_NUM_RAY_RESULT:
			if (!ctx->hasRayTest()) {
				*value = 0;
				return;
			}
			*value = ctx->getNumRayTest();
			return;
		case PL_FIND_RESULT:
			break;
		default:
		if (ctx->debug_mode) {
			ctx->debug->error("plGetDynamicWorldi(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
		break;
	}
}

void plGetDynamicWorld3f(Plenum param, Plfloat* v0, Plfloat* v1, Plfloat* v2) {
	if (!hasContext()) {
		return;
	}
	if (param == PL_GRAVITY) {
		btVector3 gravity = ctx->getWorld()->getGravity();
		*v0 = gravity.getX();
		*v1 = gravity.getY();
		*v2 = gravity.getZ();
		return;
	} else if (param == PL_RAY_POINT_RESULT) {
		if (!ctx->hasRayTest()) {
			*v0 = 0;
			*v1 = 0;
			*v2 = 0;
			if (ctx->debug_mode) {
				ctx->debug->warning("plGetDynamicWorld3f(): test a ray first");
			}
			return;
		}
		btVector3 point = ctx->getRayTest(0)->hitPoint;
		*v0 = point.getX();
		*v1 = point.getY();
		*v2 = point.getZ();
		return;
	}
	if (ctx->debug_mode) {
		ctx->debug->error("plGetDynamicWorld3f(): parameter invalid")->append(" (")->hex(param)->append(")");
	}
	REPORT_ERROR(PL_INVALID_ENUM);
}

void plGetDynamicWorldiv(Plenum param, Plint* values, Plsizei length) {
	if (!hasContext()) {
		return;
	}
	if (param == PL_RAY_BODY_RESULT) {
		if (!ctx->hasRayTest()) {
			if (ctx->debug_mode) {
				ctx->debug->error("plGetDynamicWorldiv(): there isn't a ray test");
			}
			REPORT_ERROR(PL_INVALID_OPERATION);
			return;
		}
		if (ctx->getNumRayTest() != length) {
			if (ctx->debug_mode) {
				ctx->debug->error("plGetDynamicWorldiv(): invalid length value");
			}
			REPORT_ERROR(PL_INVALID_VALUE);
			return;
		}
		for(int i = 0;i < ctx->getNumRayTest();i++) {
			values[i] = ctx->getRayTest(i)->bodyHitted;
		}
		return;
	}
	if (ctx->debug_mode) {
		ctx->debug->error("plGetDynamicWorldiv(): parameter invalid")->append(" (")->hex(param)->append(")");
	}
	REPORT_ERROR(PL_INVALID_ENUM);
}

void plGetDynamicWorldfv(Plenum param, Plfloat* values, Plsizei length) {
	if (param == PL_RAY_POINT_RESULT) {
		if (!ctx->hasRayTest()) {
			if (ctx->debug_mode) {
				ctx->debug->error("plGetDynamicWorldfv(): there isn't a ray test");
			}
			REPORT_ERROR(PL_INVALID_OPERATION);
			return;
		}
		if ((length / 3) > PL_MAX_RAY_RESULTS) {
			if (ctx->debug_mode) {
				ctx->debug->error("plGetDynamicWorldfv(): length is a lot of the maximum of results ")->pi(PL_MAX_RAY_RESULTS);
			}
			REPORT_ERROR(PL_INVALID_VALUE);
			return;
		}
		int j = 0;
		for(int i = 0;i < ctx->getNumRayTest();i++) {
			btVector3 point = ctx->getRayTest(i)->hitPoint;
			values[j] = point.getX();
			values[j+1] = point.getY();
			values[j+2] = point.getZ();
			j += 3;
		}
		return;
	} else if (param == PL_RAY_NORMAL_RESULT) {
		if (!ctx->hasRayTest()) {
			if (ctx->debug_mode) {
				ctx->debug->error("plGetDynamicWorldfv(): there isn't a ray test");
			}
			REPORT_ERROR(PL_INVALID_OPERATION);
			return;
		}
		if (ctx->getNumRayTest() != (length / 3)) {
			if (ctx->debug_mode) {
				ctx->debug->warning("plGetDynamicWorldfv(): invalid length value.");
			}
			REPORT_ERROR(PL_INVALID_VALUE);
			return;
		}
		int j = 0;
		for(int i = 0;i < ctx->getNumRayTest();i++) {
			btVector3 normal = ctx->getRayTest(i)->hitNormal;
			values[j] = normal.getX();
			values[j+1] = normal.getY();
			values[j+2] = normal.getZ();
			j += 3;
		}
		return;
	}
	if (ctx->debug_mode) {
		ctx->debug->error("plGetDynamicWorldfv(): parameter invalid")->append(" (")->hex(param)->append(")");
	}
	REPORT_ERROR(PL_INVALID_ENUM);
}

void plCreate(int target) {
	if (hasContext()) {
		if (target == PL_RIGID_BODY) {
			if (ctx->cur_body <= 0) {
				if (ctx->debug_mode) {
					ctx->debug->warning("plCreate(): rigid body name is null.");
				}
				REPORT_ERROR(PL_INVALID_NAME);
				ctx->has_global_error = true;
				return;
			}
			if (ctx->body_queue && ctx->getRigidBody(ctx->cur_body) == NULL) {
				btRigidBody* b = ctx->body_queue->make(ctx->debug);
				if (b) {
					ctx->addRigidBody(b);
					ctx->body_queue = NULL;
				} else {
					if (ctx->debug_mode) {
						ctx->debug->error("plCreate(): failure the creation of the rigid body");
					}
					ctx->has_global_error = true;
					REPORT_ERROR(PL_INVALID_VALUE);
				}
			} else {
				if (ctx->debug_mode) {
					ctx->debug->warning("plCreate(): you must have a rigid body in the queue");
				}
				ctx->has_global_error = true;
				REPORT_ERROR(PL_INVALID_OPERATION);
			}
			return;
		} else if (target == PL_TYPED_CONSTRAINT) {
			if (ctx->cur_const <= 0) {
				if (ctx->debug_mode) {
					ctx->debug->warning("plCreate(): constrain name is 0.");
				}
				REPORT_ERROR(PL_INVALID_NAME);
				ctx->has_global_error = true;
				return;
			}
			if (!ctx->const_queue) {
				if (ctx->debug_mode) {
					ctx->debug->warning("plCreate(): you must have a constraint in the queue");
				}
				ctx->has_global_error = true;
				REPORT_ERROR(PL_INVALID_OPERATION);
				return;
			}
			btTypedConstraint* constraint = ctx->const_queue->make();
			if (constraint) {
					ctx->addConstraint(constraint);
					ctx->const_queue = NULL;
				} else {
					if (ctx->debug_mode) {
						ctx->debug->error("plCreate(): failure the creation of the constraint");
					}
					ctx->has_global_error = true;
					REPORT_ERROR(PL_INVALID_VALUE);
				}
			return;
		} else if (target == PL_RAYCAST_VEHICLE) {
			if (!ctx->veh_queue) {
				if (ctx->debug_mode) {
					ctx->debug->warning("plCreate(): you must have a vehicle in the queue");
				}
				ctx->has_global_error = true;
				REPORT_ERROR(PL_INVALID_OPERATION);
				return;
			}
			btRaycastVehicle* veh = ctx->veh_queue->make(ctx->getVehRayCaster(), ctx->tuning);
			if (veh) {
				ctx->addVehicle(veh);
				ctx->veh_queue = NULL;
			} else {
				if (ctx->debug_mode) {
					ctx->debug->error("plCreate(): failure the creation of the vehicle");
				}
				ctx->has_global_error = true;
				REPORT_ERROR(PL_INVALID_VALUE);
			}
			return;
		} else if (target == PL_CHARACTER) {
			if (!ctx->char_queue) {
				if (ctx->debug_mode) {
					ctx->debug->warning("plCreate(): you must have a character in the queue");
				}
				ctx->has_global_error = true;
				REPORT_ERROR(PL_INVALID_OPERATION);
				return;
			}
			Character* ch = ctx->char_queue->make(ctx->debug);
			if (ch) {
				ctx->addCharacter(ch);
				ctx->char_queue = NULL;
			} else {
				if (ctx->debug_mode) {
					ctx->debug->error("plCreate(): failure the creation of the character");
				}
				ctx->has_global_error = true;
				REPORT_ERROR(PL_INVALID_VALUE);
			}
			return;
		}
		if (ctx->shape_queue) {
			if (ctx->cur_shape <= 0) {
				if (ctx->debug_mode) {
					ctx->debug->warning("plCreate(): shape name is invalid.");
				}
				ctx->has_global_error = true;
				REPORT_ERROR(PL_INVALID_NAME);
				return;
			}
			int result;
			btCollisionShape* shape = ctx->shape_queue->make(target,&result,ctx->debug);
			if (shape) {
				ctx->addCollisionShape(shape);
				ctx->shape_queue = NULL;
			} else {
				ctx->has_global_error = true;
				REPORT_ERROR(result);
			}
		} else {
			if (target == PL_COLLISION_SHAPE && ctx->debug_mode) {
				ctx->debug->warning("plCreate(): you must have a shape in the queue.");
			}
			REPORT_ERROR(PL_INVALID_OPERATION);
			ctx->has_global_error = true;
		}
	}
}

void plContactCallBack(obtContactCallBack callback) {
	contactCB = callback;
	if (!callback) {
		ctx->contact_testing = false;
		if (ctx->debug_mode) {
			ctx->debug->error("plContactCallBack(): callback is null, contact test is disabled");
		}
	} else {
		gContactAddedCallback = callbackFunc;
		if (ctx->debug_mode) {
			ctx->debug->info("plContactCallBack(): integred contact callback, contact test is ready.");
		}
	}
}

/*
	Functions for handle a Collision Shape
*/

Pluint plGenShape() {
	if (!hasContext()) {
		return 0;
	}
	if (ctx->shape_queue) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGenShape(): you can't create a shape.\n call btCreate() for make the current shape.");
		}
		REPORT_ERROR(PL_INVALID_OPERATION);
		return 0;
	}
	return ctx->queueShape();
}

void plBindShape(Pluint shape) {
	if (ctx->cur_shape <= 0 || shape == 0) {
		ctx->cur_shape = shape;
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plBindShape(): you can't bind this shape. unbind the previous shape");
		}
		REPORT_ERROR(PL_INVALID_OPERATION);
	}
}

void plBindBody(Pluint body) {
	if (ctx->cur_body <= 0 || body == 0) {
		ctx->cur_body = body;
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plBindBody(): you can't bind this body. unbind the previous body.");
		}
		REPORT_ERROR(PL_INVALID_OPERATION);
	}
}

void plShapei(Plenum param, Plint value) {
	if (!hasContext()) {
		return;
	}
	if (ctx->cur_shape <= 0) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plShapei(): shape name invalid.");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	switch(param) {
		case PL_USER_POINTER:
			ctx->getCollisionShape(ctx->cur_shape)->setUserPointer(reinterpret_cast<void*>(value));
			break;
		case PL_ADD_CHILD_SHAPE:
			((btCompoundShape*)ctx->getCollisionShape(ctx->cur_shape))->addChildShape(ctx->shapeTrans, ctx->getCollisionShape(value));
			break;
		case PL_REMOVE_CHILD_SHAPE:
			((btCompoundShape*)ctx->getCollisionShape(ctx->cur_shape))->removeChildShape(ctx->getCollisionShape(value));
			break;
		case PL_OPTIMIZE_CONVEX_HULL:
			ctx->shape_queue->optimizeConvexHull = (value == 1);
			break;
		default:
		if (ctx->debug_mode) {
			ctx->debug->error("plShapei(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
		break;
	}
}

void plShapef(Plenum param, Plfloat value) {
	if (!hasContext()) {
		return;
	}
	if (ctx->cur_shape == 0) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plShapef(): shape name is 0.");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_RADIUS) {
		if (ctx->shape_queue) {
			ctx->shape_queue->tmp1 = value;
			return;
		} else {
			if (ctx->debug_mode) {
				ctx->debug->error("plShapef(): the shape to assing the radius has already been created.");
			}
			REPORT_ERROR(PL_INVALID_OPERATION);
		}
	} else if (param == PL_HEIGHT) {
		if (ctx->shape_queue) {
			ctx->shape_queue->tmp2 = value;
			return;
		} else {
			if (ctx->debug_mode) {
				ctx->debug->error("plShapef(): the shape to assing the height has already been created.");
			}
			REPORT_ERROR(PL_INVALID_OPERATION);
		}
	} else if (param == PL_PLANE_CONSTANT) {
		if (ctx->shape_queue) {
			ctx->shape_queue->tmp1 = value;
			return;
		} else {
			if (ctx->debug_mode) {
				ctx->debug->error("plShapef(): the shape to assing the plane constant has already been created.");
			}
			REPORT_ERROR(PL_INVALID_OPERATION);
		}
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plShapef(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
	}
}

void plShape3f(Plenum param, Plfloat x, Plfloat y, Plfloat z) {
	if (!hasContext()) {
		return;
	}
	if (ctx->cur_shape <= 0) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plShape3f(): shape name is 0.");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_EXTENT) {
		if (ctx->shape_queue) {
			ctx->shape_queue->tmp3.setValue(x,y,z);
		} else {
			if (ctx->debug_mode) {
				ctx->debug->error("plShape3f(): the shape to assing the extent has already been created.");
			}
			REPORT_ERROR(PL_INVALID_OPERATION);
		}
	} else if (param == PL_PLANE_NORMAL) {
		if (ctx->shape_queue) {
			ctx->shape_queue->tmp3.setValue(x,y,z);
		} else {
			if (ctx->debug_mode) {
				ctx->debug->error("plShape3f(): the shape to assing the plane normal has already been created.");
			}
			REPORT_ERROR(PL_INVALID_OPERATION);
		}
	} else if (param == PL_LOCAL_SCALING) {
		ctx->getCollisionShape(ctx->cur_shape)->setLocalScaling(btVector3(x,y,z));
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plShape3f(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
	}
}

void plShapefv(Plenum param, Plfloat* values, Plsizei length) {
	if (!hasContext()) {
		return;
	}
	if (length == 0) {
		if (ctx->debug_mode) {
			ctx->debug->error("plShapefv(): invalid length value");
		}
		REPORT_ERROR(PL_INVALID_VALUE);
		return;
	}
	if (param == PL_TRANSFORM && length == 16) {
		ctx->shapeTrans.setFromOpenGLMatrix(values);
		return;
	}
	if (ctx->cur_shape > 0) {
		if (ctx->debug_mode) {
			ctx->debug->error("plShapefv(): not implemented");
		}
	} else {
		if (ctx->debug_mode) {
			ctx->debug->warning("plShapefv(): shape name is 0");
		}
		REPORT_ERROR(PL_INVALID_NAME);
	}
}

void plBufferData(Plenum type, Plsizei size, void* data, Plint copy) {
	if (type == PL_VERTEX_BUFFER) {
		ctx->shape_queue->vertex_buffer = reinterpret_cast<float*>(data);
		ctx->shape_queue->vertex_buffer_size = size;
	} else if (type == PL_INDEX_BUFFER) {
		ctx->shape_queue->index_buffer = reinterpret_cast<unsigned short*>(data);
		ctx->shape_queue->index_buffer_size = size;
	}
	ctx->shape_queue->copy_buffers = copy == PL_TRUE;
}

void plGetShapei(Pluint shape, Plenum param, Plint* value) {
	if (!hasContext()) {
		return;
	}
	if (shape <= 0) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plGetShapei(): shape name is 0.");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_USER_POINTER) {
		value = reinterpret_cast<int*>(ctx->getCollisionShape(shape)->getUserPointer());
	} else if (param == PL_SHAPE_TYPE) {
		switch(ctx->getCollisionShape(shape)->getShapeType()) {
			case BOX_SHAPE_PROXYTYPE:
				*value = PL_BOX_SHAPE;
				break;
			case SPHERE_SHAPE_PROXYTYPE:
				*value = PL_SPHERE_SHAPE;
				break;
			case CAPSULE_SHAPE_PROXYTYPE:
				*value = PL_CAPSULE_SHAPE;
				break;
			case CONE_SHAPE_PROXYTYPE:
				*value = PL_CONE_SHAPE;
				break;
			case CYLINDER_SHAPE_PROXYTYPE:
				*value = PL_CYLINDER_SHAPE;
				break;
			case STATIC_PLANE_PROXYTYPE:
				*value = PL_STATIC_PLANE_SHAPE;
				break;
			case TRIANGLE_MESH_SHAPE_PROXYTYPE:
				*value = PL_BVH_TRIANGLE_MESH_SHAPE;
				break;
			case CONVEX_HULL_SHAPE_PROXYTYPE:
				*value = PL_CONVEX_HULL_SHAPE;
				break;
			case COMPOUND_SHAPE_PROXYTYPE:
				*value = PL_COMPOUND_SHAPE;
				break;
		}
	} else if (param == PL_NUM_CHILD_SHAPES) {
		*value = ((btCompoundShape*)ctx->getCollisionShape(ctx->cur_shape))->getNumChildShapes();
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetShapei(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
	}
}

void plGetShapefv(Pluint shape, Plenum param, Plfloat* values, Plsizei length) {
	if (!hasContext()) {
		return;
	}
	if (length == 0) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetShapefv(): invalid length value");
		}
		REPORT_ERROR(PL_INVALID_VALUE);
		return;
	}
	if (shape > 0) {
		// put the parameter to get
		if (ctx->debug_mode) {
			ctx->debug->error("plGetShapefv(): not available");
		}
	} else {
		if (ctx->debug_mode) {
			ctx->debug->warning("plGetShapefv(): shape name is 0");
		}
		REPORT_ERROR(PL_INVALID_NAME);
	}
}

/*
	Functions for handle a Rigid Body
*/

Pluint plGenBody() {
	if (!hasContext()) {
		return 0;
	}
	if (ctx->body_queue) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGenBody(): you can't create a body.\n call btCreate() for make the current body.");
		}
		REPORT_ERROR(PL_INVALID_OPERATION);
		return 0;
	}
	return ctx->queueRigidBody();
}

void plDeleteBody(Pluint body) {
	if (!hasContext()) {
		return;
	}
	if (body <= 0 || ctx->cur_body == body) {
		if (ctx->debug_mode) {
			ctx->debug->error("plDeleteBody(): cause 1: you can't delete the current body.\ncause 2: body name is 0");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	ctx->deleteRigidBody(body);
}

void plRigidBodyi(Plenum param, Plint value) {
	if (!hasContext()) {
		return;
	}
	if (ctx->cur_body <= 0) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plRigidBodyi(): body name is 0.");
		}
		ctx->has_global_error = true;
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	btRigidBody* rb = ctx->getRigidBody(ctx->cur_body);
	if (!ctx->body_queue && rb == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plRigidBodyi(): rigid body not exist.");
		}
		ctx->has_global_error = true;
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_USER_POINTER) {
		if (!ctx->contact_testing) {
			rb->setUserPointer(reinterpret_cast<void*>(value));
		} else {
			ContactCBInfo* inf = new ContactCBInfo();
			inf->user_ptr = value;
			rb->setUserPointer((void*)inf);
		}
	} else if (param == PL_COLLISION_CALLBACK_FILTER) {
		if (ctx->contact_testing && rb->getUserPointer() != NULL) {
			((ContactCBInfo*)rb->getUserPointer())->col_filter = value;
		} else {
			REPORT_ERROR(PL_INVALID_OPERATION);
			return;
		}
	} else if (param == PL_COLLISION_CALLBACK_FLAG) {
		if (ctx->contact_testing && rb->getUserPointer() != NULL) {
			((ContactCBInfo*)rb->getUserPointer())->col_flag = value;
		} else {
			REPORT_ERROR(PL_INVALID_OPERATION);
			return;
		}
	} else if (param == PL_USER_INDEX) {
		rb->setUserIndex(value);
	} else if (param == PL_ACTIVATION_STATE) {
		switch(value) {
			case PL_DIS_DACTIVATION:
				rb->setActivationState(DISABLE_DEACTIVATION);
				break;
			case PL_DIS_SIMULATION:
				rb->setActivationState(DISABLE_SIMULATION);
				break;
			case PL_ACTIVATE:
				rb->activate();
				break;
		}
	} else if (param == PL_ADD_COLLISION_FLAG) {
		switch(value) {
			case PL_CF_KINEMATIC_OBJECT:
				rb->setCollisionFlags(rb->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
				break;
			case PL_CF_STATIC_OBJECT:
				rb->setCollisionFlags(rb->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
				break;
			case PL_CF_CUSTOM_MATERIAL_CALLBACK:
				rb->setCollisionFlags(rb->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
				break;
		}
	} else if (param == PL_COLLISION_SHAPE) {
		if (ctx->body_queue) {
			ctx->body_queue->shape = ctx->getCollisionShape(value);
		} else {
			rb->setCollisionShape(ctx->getCollisionShape(value));
		}
	} else if (param == PL_ANISOTROPIC_ROLLING_FRICTION) {
		btCollisionShape* shape = rb->getCollisionShape();
		rb->setAnisotropicFriction(shape->getAnisotropicRollingFrictionDirection(),btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
	} else if (param == PL_CLEAR_FORCES) {
		rb->clearForces();
		if (value == PL_ANGULAR_LINEAR_VEL) {
			btVector3 zeroVec(0.0f, 0.0f, 0.0f);
			rb->setLinearVelocity(zeroVec);
			rb->setAngularVelocity(zeroVec);
		}
	} else{
		if (ctx->debug_mode) {
			ctx->debug->error("plRigidBodyi(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
	}
}

void plRigidBodyf(Plenum param, Plfloat value) {
	if (!hasContext()) {
		return;
	}
	
	if (ctx->cur_body <= 0) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plRigidBodyf(): body name is 0.");
		}
		ctx->has_global_error = true;
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	
	btRigidBody* rb = ctx->getRigidBody(ctx->cur_body);
	if (!ctx->body_queue && rb == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plRigidBodyf(): rigid body not exist.");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_MASS) {
		if (ctx->body_queue) {
			if (!ctx->body_queue->shape) {
				REPORT_ERROR(PL_INVALID_OPERATION);
				return;
			}
			btCollisionShape* sp = ctx->body_queue->shape;
			btVector3 inertia(0,0,0);
			if (value != 0.0f) {
				sp->calculateLocalInertia(value,inertia);
			}
			ctx->body_queue->inertia = inertia;
			ctx->body_queue->mass = value;
		} else {
			btCollisionShape* sp = rb->getCollisionShape();
			btVector3 inertia(0,0,0);
			if (value != btScalar(0)) {
				sp->calculateLocalInertia(value,inertia);
			}
			rb->setMassProps(value, inertia);
		}
	} else if (param == PL_ANGULAR_FACTOR) {
		rb->setAngularFactor(value);
	} else if (param == PL_ROLLING_FRICTION) {
		rb->setRollingFriction(value);
	} else if (param == PL_SPINNING_FRICTION) {
		rb->setSpinningFriction(value);
	} else if (param == PL_FRICTION) {
		rb->setFriction(value);
	} else if (param == PL_RESTITUTION) {
		rb->setRestitution(value);
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plRigidBodyf(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
	}
}

void plRigidBody3f(Plenum param, Plfloat x, Plfloat y, Plfloat z) {
	if (!hasContext()) {
		return;
	}
	if (ctx->cur_body <= 0) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plRigidBody3f(): body name is 0.");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	btRigidBody* rb = ctx->getRigidBody(ctx->cur_body);
	if (param != PL_MOTION_STATE_POSITION && rb == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plRigidBody3f(): rigid body not exist.");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	
	if (param == PL_POSITION) {
		rb->getWorldTransform().getOrigin().setValue(x, y, z);
	} else if (param == PL_MOTION_STATE_POSITION) {
		if (ctx->body_queue) {
			if (!ctx->body_queue->motion) {
				btTransform t;
				t.setIdentity();
				t.setOrigin(btVector3(x,y,z));
				ctx->body_queue->motion = new btDefaultMotionState(t);
			} else {
				if (ctx->debug_mode) {
					ctx->debug->warning("plRigidBody3f(): You can only call once this parameter");
				}
				REPORT_ERROR(PL_INVALID_OPERATION);
				return;
			}
		} else {
			if (ctx->debug_mode) {
				ctx->debug->warning("plRigidBody3f(): MOTION_STATE_POSITION not available");
			}
			REPORT_ERROR(PL_INVALID_OPERATION);
			return;
		}
	} else if (param == PL_DAMPING) {
		rb->setDamping(x,y);
	} else if (param == PL_LINEAR_FACTOR) {
		rb->setLinearFactor(btVector3(x,y,z));
	} else if (param == PL_ANGULAR_FACTOR) {
		rb->setAngularFactor(btVector3(x,y,z));
	} else if (param == PL_LOCAL_INERTIA) {
		btScalar invMass = rb->getInvMass();
		rb->setMassProps(btScalar(1.0f) / invMass,btVector3(x,y,z));
	} else if (param == PL_APPLY_CENTRAL_FORCE) {
		rb->applyCentralForce(btVector3(x,y,z));
	} else if (param == PL_APPLY_TORQUE) {
		rb->applyTorque(btVector3(x,y,z));
	} else if (param == PL_APPLY_CENTRAL_IMPULSE) {
		rb->applyCentralImpulse(btVector3(x,y,z));
	} else if (param == PL_APPLY_TORQUE_IMPULSE) {
		rb->applyTorqueImpulse(btVector3(x,y,z));
	} else if (param == PL_ANISOTROPIC_FRICTION) {
		rb->setAnisotropicFriction(btVector3(x,y,z),btCollisionObject::CF_ANISOTROPIC_FRICTION);
	} else if (param == PL_COL_INTERP_LINEAR_VEL) {
		rb->setInterpolationLinearVelocity(btVector3(x,y,z));
	} else if (param == PL_COL_INTERP_ANGULAR_VEL) {
		rb->setInterpolationAngularVelocity(btVector3(x,y,z));
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plRigidBody3f(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
	}
}

void plRigidBodyfv(Plenum param, Plfloat* values, Plsizei length) {
	if (!hasContext()) {
		return;
	}
	if (ctx->cur_body <= 0) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plRigidBodyfv(): body name is 0.");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	int numLen = getParamLen(param);
	if (numLen == -1) {
		return;
	}
	if (length == 0 ||  length != numLen) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plRigidBodyfv(): invalid length value, use length: ")->pi(numLen)->append(" and try again");
		}
		REPORT_ERROR(PL_INVALID_VALUE);
		return;
	}
	btRigidBody* rb = ctx->getRigidBody(ctx->cur_body);
	if (!ctx->body_queue && rb == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plRigidBodyfv(): rigid body not exist.");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_POSITION) {
		rb->getWorldTransform().getOrigin().setValue(values[0],values[1],values[2]);
	} else if (param == PL_ORIENTATION) {
		rb->getWorldTransform().getRotation().setValue(values[0],values[1],values[2],values[3]);
	} else if (param == PL_LINEAR_FACTOR) {
		rb->setLinearFactor(btVector3(values[0],values[1],values[2]));
	} else if (param == PL_ANGULAR_FACTOR) {
		rb->setAngularFactor(btVector3(values[0],values[1],values[2]));
	} else if (param == PL_LOCAL_INERTIA) {
		btScalar invMass = rb->getInvMass();
		rb->setMassProps(1.0f / invMass,btVector3(values[0],values[1],values[2]));
	} else if (param == PL_APPLY_CENTRAL_FORCE) {
		rb->applyCentralForce(btVector3(values[0],values[1],values[2]));
	} else if (param == PL_APPLY_TORQUE) {
		rb->applyTorque(btVector3(values[0],values[1],values[2]));
	} else if (param == PL_APPLY_FORCE) {
		rb->applyForce(btVector3(values[0],values[1],values[2]),btVector3(values[3],values[4],values[5]));
	} else if (param == PL_APPLY_IMPULSE) {
		rb->applyImpulse(btVector3(values[0],values[1],values[2]),btVector3(values[3],values[4],values[5]));
	} else if (param == PL_APPLY_CENTRAL_IMPULSE) {
		rb->applyCentralImpulse(btVector3(values[0],values[1],values[2]));
	} else if (param == PL_APPLY_TORQUE_IMPULSE) {
		rb->applyTorqueImpulse(btVector3(values[0],values[1],values[2]));
	} else if (param == PL_BASIS) {
		btTransform t;
		t.setIdentity();
		t.getBasis().setFromOpenGLSubMatrix(values);
		rb->proceedToTransform(t);
	} else if (param == PL_TRANSFORM) {
		btTransform t;
		t.setIdentity();
		t.setFromOpenGLMatrix(values);
		rb->proceedToTransform(t);
	} else if (param == PL_MOTION_STATE_TRANSFORM) {
		if (ctx->body_queue) {
			if (!ctx->body_queue->motion) {
				btTransform t;
				t.setFromOpenGLMatrix(values);
				ctx->body_queue->motion = new btDefaultMotionState(t);
			} else {
				if (ctx->debug_mode) {
					ctx->debug->warning("plRigidBodyfv(): You can only call one times this parameter");
				}
				REPORT_ERROR(PL_INVALID_OPERATION);
				return;
			}
		} else {
			if (ctx->debug_mode) {
				ctx->debug->warning("plRigidBodyfv(): MOTION_STATE_TRANSFORM not available");
			}
			REPORT_ERROR(PL_INVALID_OPERATION);
			return;
		}
	} else if (param == PL_MOTION_STATE_POSITION) {
		if (ctx->body_queue) {
			if (!ctx->body_queue->motion) {
				btTransform t;
				t.setIdentity();
				t.setOrigin(btVector3(values[0],values[1],values[2]));
				ctx->body_queue->motion = new btDefaultMotionState(t);
			} else {
				if (ctx->debug_mode) {
					ctx->debug->warning("plRigidBodyfv(): You can only call one times this parameter");
				}
				REPORT_ERROR(PL_INVALID_OPERATION);
				return;
			}
		} else {
			if (ctx->debug_mode) {
				ctx->debug->warning("plRigidBodyfv(): MOTION_STATE_TRANSFORM not available");
			}
			REPORT_ERROR(PL_INVALID_OPERATION);
			return;
		}
	} else if (param == PL_COL_WORLD_TRANSFORM) {
		btTransform t;
		t.setIdentity();
		t.setFromOpenGLMatrix(values);
		rb->setWorldTransform(t);
	} else if (param == PL_COL_INTERP_WORLD_TRANSFORM) {
		btTransform t;
		t.setIdentity();
		t.setFromOpenGLMatrix(values);
		rb->setInterpolationWorldTransform(t);
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plRigidBodyfv(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
	}
}

void plGetRigidBodyi(Pluint body, Plenum param, Plint* value) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (body <= 0) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plGetRigidBodyi(): body name is 0.");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	btRigidBody* rb = ctx->getRigidBody(body);
	if (!ctx->body_queue && rb == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetRigidBodyi(): rigid body not exist.");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_USER_POINTER) {
		if (!ctx->contact_testing) {
			value = reinterpret_cast<int*>(rb->getUserPointer());
		} else {
			*value = ((ContactCBInfo*)rb->getUserPointer())->user_ptr;
		}
	} else if (param == PL_USER_INDEX) {
		*value = rb->getUserIndex();
	} else if (param == PL_IS_IN_WORLD) {
		*value = rb->isInWorld();
	} else if (param == PL_ACTIVATION_STATE) {
		switch(rb->getActivationState()) {
			case DISABLE_DEACTIVATION: 
				*value = PL_DIS_DACTIVATION; 
				break;
			case DISABLE_SIMULATION: 
				*value = PL_DIS_SIMULATION; 
				break;
			case ISLAND_SLEEPING: 
				*value = PL_ISLAND_SLEEPING; 
				break;
			case ACTIVE_TAG: 
				*value = PL_ACTIVATE; 
				break;
		}
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetRigidBodyi(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
	}
}

void plGetRigidBodyf(Pluint body, Plenum param, Plfloat* value) {
	if (!hasContext()) {
		return;
	}
	if (body <= 0) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plGetRigidBodyf(): body name is 0");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	btRigidBody* rb = ctx->getRigidBody(body);
	if (!ctx->body_queue && rb == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetRigidBodyf(): rigid body not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	
	if (param == PL_MASS) {
		*value = 1.0f / rb->getInvMass();
	} else if (param == PL_LINEAR_DAMPING) {
		*value = rb->getLinearDamping();
	} else if (param == PL_ANGULAR_DAMPING) {
		*value = rb->getAngularDamping();
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetRigidBodyf(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
	}
}
void plGetRigidBody3f(Pluint body, Plenum param, Plfloat* v1, Plfloat* v2, Plfloat* v3) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (body <= 0) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plGetRigidBody3f(): body name is 0");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	btRigidBody* rb = ctx->getRigidBody(body);
	if (!ctx->body_queue && rb == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetRigidBody3f(): rigid body not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_POSITION) {
		btVector3 pos = rb->getCenterOfMassPosition();
		*v1 = pos.getX();
		*v2 = pos.getY();
		*v3 = pos.getZ();
	} else if (param == PL_DAMPING) {
		*v1 = rb->getLinearDamping();
		*v2 = rb->getAngularDamping();
		*v3 = 0.0f;
	} else if (param == PL_LINEAR_FACTOR) {
		btVector3 fact = rb->getLinearFactor();
		*v1 = fact.getX();
		*v2 = fact.getY();
		*v3 = fact.getZ();
	} else if (param == PL_ANGULAR_FACTOR) {
		btVector3 fact = rb->getAngularFactor();
		*v1 = fact.getX();
		*v2 = fact.getY();
		*v3 = fact.getZ();
	} else if (param == PL_LOCAL_INERTIA) {
		btVector3 inertia = rb->getLocalInertia();
		*v1 = inertia.getX();
		*v2 = inertia.getY();
		*v3 = inertia.getZ();
	} else if (param == PL_TOTAL_TORQUE) {
		btVector3 torque = rb->getTotalTorque();
		*v1 = torque.getX();
		*v2 = torque.getY();
		*v3 = torque.getZ();
	} else if (param == PL_TOTAL_FORCE) {
		btVector3 force = rb->getTotalForce();
		*v1 = force.getX();
		*v2 = force.getY();
		*v3 = force.getZ();
	} else if (param == PL_LINEAR_VELOCITY) {
		btVector3 l_vel = rb->getLinearVelocity();
		*v1 = l_vel.getX();
		*v2 = l_vel.getY();
		*v3 = l_vel.getZ();
	} else if (param == PL_ANGULAR_VELOCITY) {
		btVector3 a_vel = rb->getAngularVelocity();
		*v1 = a_vel.getX();
		*v2 = a_vel.getY();
		*v3 = a_vel.getZ();
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetRigidBody3f(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
	}
}

void plGetRigidBodyfv(Pluint body, Plenum param, Plfloat* values, Plsizei length) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (ctx->body_queue) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetRigidBodyfv(): call btCreate(PL_RIGID_BODY) and try again.");
		}
		REPORT_ERROR(PL_INVALID_OPERATION);
		return;
	}
	if (body <= 0) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plGetRigidBodyfv(): body name is 0");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	int numLen = getParamLen(param);
	if (numLen == -1) {
		return;
	}
	if (length == 0 ||  length != numLen) {
		if (ctx->debug_mode) {
			ctx->debug->warning("plGetRigidBodyfv(): invalid length value, use length: ")->pi(numLen)->append(" and try again");
		}
		REPORT_ERROR(PL_INVALID_VALUE);
		return;
	}
	btRigidBody* rb = ctx->getRigidBody(body);
	if (rb == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetRigidBodyfv(): rigid body not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_POSITION) {
		btVector3 pos = rb->getCenterOfMassPosition();
		values[0] = pos.getX();
		values[1] = pos.getY();
		values[2] = pos.getZ();
	} else if (param == PL_ORIENTATION) {
		btQuaternion q = rb->getOrientation();
		values[0] = q.getX();
		values[1] = q.getY();
		values[2] = q.getZ();
		values[3] = q.getW();
	} else if (param == PL_LINEAR_FACTOR) {
		btVector3 fact = rb->getLinearFactor();
		values[0] = fact.getX();
		values[1] = fact.getY();
		values[2] = fact.getZ();
	} else if (param == PL_ANGULAR_FACTOR) {
		btVector3 fact = rb->getAngularFactor();
		values[0] = fact.getX();
		values[1] = fact.getY();
		values[2] = fact.getZ();
	} else if (param == PL_LOCAL_INERTIA) {
		btVector3 inertia = rb->getLocalInertia();
		values[0] = inertia.getX();
		values[1] = inertia.getY();
		values[2] = inertia.getZ();
	} else if (param == PL_TOTAL_TORQUE) {
		btVector3 torque = rb->getTotalTorque();
		values[0] = torque.getX();
		values[1] = torque.getY();
		values[2] = torque.getZ();
	} else if (param == PL_TOTAL_FORCE) {
		btVector3 force = rb->getTotalForce();
		values[0] = force.getX();
		values[1] = force.getY();
		values[2] = force.getZ();
	} else if (param == PL_LINEAR_VELOCITY) {
		btVector3 l_vel = rb->getLinearVelocity();
		values[0] = l_vel.getX();
		values[1] = l_vel.getY();
		values[2] = l_vel.getZ();
	} else if (param == PL_ANGULAR_VELOCITY) {
		btVector3 a_vel = rb->getAngularVelocity();
		values[0] = a_vel.getX();
		values[1] = a_vel.getY();
		values[2] = a_vel.getZ();
	} else if (param == PL_TRANSFORM) {
		const btTransform t = rb->getCenterOfMassTransform();
		t.getOpenGLMatrix(values);
	} else if (param == PL_BASIS) {
		const btTransform t = rb->getCenterOfMassTransform();
		t.getBasis().getOpenGLSubMatrix(values);
		values[12] = 0.0f; 
		values[13] = 0.0f; 
		values[14] = 0.0f;
		values[15] = 1.0f;
	} else if (param == PL_MOTION_STATE_TRANSFORM) {
		btTransform t;
		rb->getMotionState()->getWorldTransform(t);
		t.getOpenGLMatrix(values);
	} else if (param == PL_MOTION_STATE_POSITION) {
		btTransform t;
		rb->getMotionState()->getWorldTransform(t);
		btVector3 pos = t.getOrigin();
		values[0] = pos.getX();
		values[1] = pos.getY();
		values[2] = pos.getZ();
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetRigidBodyfv(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
	}
}

/*
	Functions to handle constraints
*/

Pluint plGenConstraint(Plenum type) {
	if (!hasContext() || ctx->has_global_error) {
		return 0;
	}
	if (ctx->const_queue) {
		REPORT_ERROR(PL_INVALID_OPERATION);
		return 0;
	}
	return ctx->queueConstraint(type);
}

void plBindConstraint(Pluint ctr) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (ctx->cur_const <= 0 || ctr == 0) {
		ctx->cur_const = ctr;
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plBindConstraint(): you can't bind this constraint. unbind the previous constraint");
		}
		REPORT_ERROR(PL_INVALID_OPERATION);
	}
}

void plDeleteConstraint(Pluint ctr) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	ctx->deleteConstraint(ctr);
}

void plConstrainti(Plenum param, Plint value) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (ctx->const_queue) {
		switch(param) {
			case PL_CONSTR_RBODY_A:
				if (value <= 0) {
					break;
				}
				ctx->const_queue->rbA = ctx->getRigidBody(value);
				break;
			case PL_CONSTR_RBODY_B:
				if (value <= 0) {
					break;
				}
				ctx->const_queue->rbB = ctx->getRigidBody(value);
				break;
			case PL_CONSTR_USE_REFERENCE_A:
				ctx->const_queue->useRefA = (value == 1);
				break;
		}
		return;
	}
	btTypedConstraint* tmp = ctx->getConstraint(ctx->cur_const);
	if (tmp == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plConstrainti(): constraint not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	switch(param) {
		case PL_HC_MAX_MOTOR_IMPULSE:
			((btHingeConstraint*)tmp)->setAngularOnly(value == 1);
			break;
		case PL_HC_ENABLE_ANGULAR_MOTOR:
			((btHingeConstraint*)tmp)->enableMotor(value == 1);
			break;
		case PL_HC_ANGULAR_ONLY:
			((btHingeConstraint*)tmp)->setAngularOnly(value == 1);
			break;
		case PL_G6DS_ENABLE_SPRING:
			((btGeneric6DofSpringConstraint*)tmp)->enableSpring(value, true);
			break;
		case PL_G6DS_DISABLE_SPRING:
			((btGeneric6DofSpringConstraint*)tmp)->enableSpring(value, false);
			break;
		case PL_G6DS_EQUILI_POINT_SPRING:
			if (value == -1) {
				((btGeneric6DofSpringConstraint*)tmp)->setEquilibriumPoint();
				return;
			}
			((btGeneric6DofSpringConstraint*)tmp)->setEquilibriumPoint(value);
			break;
		case PL_G6DS2_ENABLE_SPRING:
			((btGeneric6DofSpring2Constraint*)tmp)->enableSpring(value, true);
			break;
		case PL_G6DS2_DISABLE_SPRING:
			((btGeneric6DofSpring2Constraint*)tmp)->enableSpring(value, false);
			break;
		case PL_G6DS2_EQUILI_POINT_SPRING:
			if (value == -1) {
				((btGeneric6DofSpring2Constraint*)tmp)->setEquilibriumPoint();
				return;
			}
			((btGeneric6DofSpring2Constraint*)tmp)->setEquilibriumPoint(value);
			break;
		case PL_G6DS2_ENABLE_MOTOR:
			((btGeneric6DofSpring2Constraint*)tmp)->enableMotor(value, true);
			break;
		case PL_G6DS2_SET_SERVO:
			((btGeneric6DofSpring2Constraint*)tmp)->setServo(value, true);
			break;
		case PL_SLDR_POWERED_LINEAR_MOTOR:
			((btSliderConstraint*)tmp)->setPoweredLinMotor(value);
			break;
		case PL_SLDR_POWERED_ANGULAR_MOTOR:
			((btSliderConstraint*)tmp)->setPoweredAngMotor(value);
			break;
		default:
		if (ctx->debug_mode) {
			ctx->debug->error("plConstrainti(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
		break;
	}
}

void plConstraintf(Plenum param, Plfloat value) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (ctx->const_queue) {
		REPORT_ERROR(PL_INVALID_OPERATION);
		return;
	}
	btTypedConstraint* tmp = ctx->getConstraint(ctx->cur_const);
	if (tmp == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plConstraintf(): constraint not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	switch(param) {
		case PL_HC_MAX_MOTOR_IMPULSE:
			((btHingeConstraint*)tmp)->setMaxMotorImpulse(value);
			break;
		case PL_HC_TARGET_VELOCITY:
			((btHingeConstraint*)tmp)->setMotorTargetVelocity(value);
			break;
		case PL_SLDR_SOFTNESS_DIR_LINEAR:
			((btSliderConstraint*)tmp)->setSoftnessDirLin(value);
			break;
		case PL_SLDR_SOFTNESS_DIR_ANGULAR:
			((btSliderConstraint*)tmp)->setSoftnessDirAng(value);
			break;
		case PL_SLDR_SOFTNESS_LIMIT_LIN:
			((btSliderConstraint*)tmp)->setSoftnessLimLin(value);
			break;
		case PL_SLDR_SOFTNESS_ORTHO_LIN:
			((btSliderConstraint*)tmp)->setSoftnessOrthoLin(value);
			break;
		case PL_SLDR_SOFTNESS_ORTHO_ANG:
			((btSliderConstraint*)tmp)->setSoftnessOrthoAng(value);
			break;
		case PL_SLDR_DAMPING_DIR_LINEAR:
			((btSliderConstraint*)tmp)->setDampingDirLin(value);
			break;
		case PL_SLDR_DAMPING_DIR_ANGULAR:
			((btSliderConstraint*)tmp)->setDampingDirAng(value);
			break;
		case PL_SLDR_DAMPING_LIMIT_LIN:
			((btSliderConstraint*)tmp)->setDampingLimLin(value);
			break;
		case PL_SLDR_DAMPING_ORTHO_LIN:
			((btSliderConstraint*)tmp)->setDampingOrthoLin(value);
			break;
		case PL_SLDR_DAMPING_ORTHO_ANG:
			((btSliderConstraint*)tmp)->setDampingOrthoAng(value);
			break;
		case PL_SLDR_RESTITUTION_DIR_LINEAR:
			((btSliderConstraint*)tmp)->setRestitutionDirLin(value);
			break;
		case PL_SLDR_RESTITUTION_DIR_ANGULAR:
			((btSliderConstraint*)tmp)->setRestitutionDirAng(value);
			break;
		case PL_SLDR_RESTITUTION_LIMIT_LIN:
			((btSliderConstraint*)tmp)->setRestitutionLimLin(value);
			break;
		case PL_SLDR_RESTITUTION_ORTHO_LIN:
			((btSliderConstraint*)tmp)->setRestitutionOrthoLin(value);
			break;
		case PL_SLDR_RESTITUTION_ORTHO_ANG:
			((btSliderConstraint*)tmp)->setRestitutionOrthoAng(value);
			break;
		case PL_SLDR_MAX_ANG_MOTOR_FORCE:
			((btSliderConstraint*)tmp)->setMaxAngMotorForce(value);
			break;
		case PL_SLDR_MAX_LIN_MOTOR_FORCE:
			((btSliderConstraint*)tmp)->setMaxLinMotorForce(value);
			break;
		case PL_SLDR_TARGET_LIN_MOTOR_VEL:
			((btSliderConstraint*)tmp)->setTargetLinMotorVelocity(value);
			break;
		case PL_SLDR_TARGET_ANG_MOTOR_VEL:
			((btSliderConstraint*)tmp)->setTargetAngMotorVelocity(value);
			break;
		case PL_HC2_LOWER_LIMIT:
			((btHinge2Constraint*)tmp)->setLowerLimit(value);
			break;
		case PL_HC2_UPPER_LIMIT:
			((btHinge2Constraint*)tmp)->setUpperLimit(value);
			break;
		case PL_P2P_UPDATE_RHS:
			((btPoint2PointConstraint*)tmp)->updateRHS(value);
			break;
			default:
		if (ctx->debug_mode) {
			ctx->debug->error("plConstraintf(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
		break;
	}
}

void plConstraint3f(Plenum param, Plfloat x, Plfloat y, Plfloat z) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (ctx->const_queue) {
		switch(param) {
			case PL_CONSTR_PIVOT_A:
				ctx->const_queue->tempA.setValue(x,y,z);
				break;
			case PL_CONSTR_PIVOT_B:
				ctx->const_queue->tempB.setValue(x,y,z);
				break;
			case PL_CONSTR_IN_AXIS_A:
				ctx->const_queue->tempC.setValue(x,y,z);
				break;
			case PL_CONSTR_IN_AXIS_B:
				ctx->const_queue->tempD.setValue(x,y,z);
				break;
		}
		return;
	}
	btTypedConstraint* tmp = ctx->getConstraint(ctx->cur_const);
	if (tmp == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plConstraint3f(): constraint not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_CTWIST_LIMITS) {
		((btConeTwistConstraint*)tmp)->setLimit(x,y,z);
		return;
	}
	switch(param) {
		case PL_HC_LIMITS:
			((btHingeConstraint*)tmp)->setLimit(x,y);
			break;
		case PL_HC2_LIMITS:
			((btHinge2Constraint*)tmp)->setUpperLimit(x);
			((btHinge2Constraint*)tmp)->setLowerLimit(y);
			break;
		case PL_G6D_LOWER_LIN_LIMIT:
			((btGeneric6DofConstraint*)tmp)->setLinearLowerLimit(btVector3(x,y,z));
			break;
		case PL_G6D_UPPER_LIN_LIMIT:
			((btGeneric6DofConstraint*)tmp)->setLinearUpperLimit(btVector3(x,y,z));
			break;
		case PL_G6D_LOWER_ANG_LIMIT:
			((btGeneric6DofConstraint*)tmp)->setAngularLowerLimit(btVector3(x,y,z));
			break;
		case PL_G6D_UPPER_ANG_LIMIT:
			((btGeneric6DofConstraint*)tmp)->setAngularUpperLimit(btVector3(x,y,z));
			break;
		case PL_G6D_LIMIT_LIN_X:
			((btGeneric6DofConstraint*)tmp)->setLimit(0,x,y);
			break;
		case PL_G6D_LIMIT_LIN_Y:
			((btGeneric6DofConstraint*)tmp)->setLimit(1,x,y);
			break;
		case PL_G6D_LIMIT_LIN_Z:
			((btGeneric6DofConstraint*)tmp)->setLimit(2,x,y);
			break;
		case PL_G6D_LIMIT_ANG_X:
			((btGeneric6DofConstraint*)tmp)->setLimit(3,x,y);
			break;
		case PL_G6D_LIMIT_ANG_Y:
			((btGeneric6DofConstraint*)tmp)->setLimit(4,x,y);
			break;
		case PL_G6D_LIMIT_ANG_Z:
			((btGeneric6DofConstraint*)tmp)->setLimit(5,x,y);
			break;
		case PL_G6DS_STIFFNESS_SPRING:
			((btGeneric6DofSpringConstraint*)tmp)->setStiffness(int(x),y);
			break;
		case PL_G6DS_DAMPING_SPRING:
			((btGeneric6DofSpringConstraint*)tmp)->setDamping(int(x),y);
			break;
		case PL_G6DS_EQUILI_POINT_SPRING:
			((btGeneric6DofSpringConstraint*)tmp)->setEquilibriumPoint(int(x),y);
			break;
		case PL_G6DS2_LOWER_LIN_LIMIT:
			((btGeneric6DofSpring2Constraint*)tmp)->setLinearLowerLimit(btVector3(x,y,z));
			break;
		case PL_G6DS2_UPPER_LIN_LIMIT:
			((btGeneric6DofSpring2Constraint*)tmp)->setLinearUpperLimit(btVector3(x,y,z));
			break;
		case PL_G6DS2_LOWER_ANG_LIMIT:
			((btGeneric6DofSpring2Constraint*)tmp)->setAngularLowerLimit(btVector3(x,y,z));
			break;
		case PL_G6DS2_UPPER_ANG_LIMIT:
			((btGeneric6DofSpring2Constraint*)tmp)->setAngularUpperLimit(btVector3(x,y,z));
			break;
		case PL_G6DS2_STIFFNESS_SPRING:
			((btGeneric6DofSpring2Constraint*)tmp)->setStiffness(int(x),y);
			break;
		case PL_G6DS2_DAMPING_SPRING:
			((btGeneric6DofSpring2Constraint*)tmp)->setDamping(int(x),y);
			break;
		case PL_G6DS2_EQUILI_POINT_SPRING:
			((btGeneric6DofSpring2Constraint*)tmp)->setEquilibriumPoint(int(x),y);
			break;
		case PL_G6DS2_TARGET_VELOCITY:
			((btGeneric6DofSpring2Constraint*)tmp)->setTargetVelocity(int(x),y);
			break;
		case PL_G6DS2_SERVO_TARGET:
			((btGeneric6DofSpring2Constraint*)tmp)->setServoTarget(int(x),y);
			break;
		case PL_G6DS2_MAX_MOTOR_FORCE:
			((btGeneric6DofSpring2Constraint*)tmp)->setMaxMotorForce(int(x),y);
			break;
		case PL_G6DS2_BOUNCE:
			((btGeneric6DofSpring2Constraint*)tmp)->setBounce(int(x),y);
			break;
		case PL_P2P_PIVOT_A:
			((btPoint2PointConstraint*)tmp)->setPivotA(btVector3(x,y,z));
			break;
		case PL_P2P_PIVOT_B:
			((btPoint2PointConstraint*)tmp)->setPivotB(btVector3(x,y,z));
			break;
		default:
		if (ctx->debug_mode) {
			ctx->debug->error("plConstraint3f(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
		break;
	}
}

void plConstraintfv(Plenum param, Plfloat* values, Plsizei length) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (ctx->const_queue) {
		if (length != 16) {
			return;
		}
		switch(param) {
			case PL_CONSTR_FRAME_A:
				ctx->const_queue->transA.setFromOpenGLMatrix(values);
				break;
			case PL_CONSTR_FRAME_B:
				ctx->const_queue->transB.setFromOpenGLMatrix(values);
				break;
		}
		return;
	}
}

/*
		Functions to handle vehicles
*/
Pluint plGenVehicle() {
	if (!hasContext()) {
		return 0;
	}
	if (ctx->veh_queue) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGenVehicle(): you can't create a vehicle.\n call btCreate() for make the current vehicle.");
		}
		REPORT_ERROR(PL_INVALID_OPERATION);
		return 0;
	}
	return ctx->queueVehicle();
}

void plBindVehicle(Pluint indx) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (ctx->cur_veh <= 0 || indx == 0) {
		ctx->cur_veh = indx;
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plBindVehicle(): you can't bind this vehicle. unbind the previous vehicle");
		}
		REPORT_ERROR(PL_INVALID_OPERATION);
	}
}

void plDeleteVehicle(Pluint indx) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	ctx->deleteVehicle(indx);
}

void plWheelf(Plint wheel, Plenum param, Plfloat value) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	btRaycastVehicle* tmp = ctx->getVehicle(ctx->cur_veh);
	if (tmp == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plWheelf(): vehicle not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}

	btWheelInfo& info = tmp->getWheelInfo(wheel);
	switch(param) {
		case PL_WHEEL_ENGINE_FORCE:
			info.m_engineForce = value;
			break;
		case PL_WHEEL_STEERING:
			info.m_steering = value;
			break;
		case PL_WHEEL_BRAKE:
			info.m_brake = value;
			break;
		case PL_WHEEL_ROLL_INFLUENCE:
			info.m_rollInfluence = value;
			break;
		default:
		if (ctx->debug_mode) {
			ctx->debug->error("plWheelf(): parameter invalid")->append(" (")->hex(param)->append(")");
		}
		REPORT_ERROR(PL_INVALID_ENUM);
		break;
	}
}

void plVehiclei(Plenum param, Plint value) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (ctx->veh_queue) {
		switch(param) {
			case PL_VEHICLE_CHASSIS:
				ctx->veh_queue->chassis = ctx->getRigidBody(value);
				break;
			default:
			if (ctx->debug_mode) {
				ctx->debug->error("plVehiclei(): parameter invalid")->append(" (")->hex(param)->append(")");
			}
			REPORT_ERROR(PL_INVALID_ENUM);
			break;
		}
		return;
	}
}

void plVehiclef(Plenum param, Plfloat value) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	switch(param) {
		case PL_SUSPENSION_STIFFNESS:
			ctx->tuning.m_suspensionStiffness = value;
			break;
		case PL_SUSPENSION_DAMPING:
			ctx->tuning.m_suspensionDamping = value;
			break;
		case PL_SUSPENSION_COMPRESSION:
			ctx->tuning.m_suspensionCompression = value;
			break;
		case PL_MAX_SUSPENSION_TRAVEL:
			ctx->tuning.m_maxSuspensionTravelCm = value;
			break;
		case PL_FRICTION_SLIP:
			ctx->tuning.m_frictionSlip = value;
			break;
		case PL_MAX_SUSPENSION_FORCE:
			ctx->tuning.m_maxSuspensionForce = value;
			break;
	}
}

void plVehicle3f(Plenum param, Plfloat x, Plfloat y, Plfloat z) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	btRaycastVehicle* tmp = ctx->getVehicle(ctx->cur_veh);
	if (tmp == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plVehicle3f(): vehicle not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	switch(param) {
		case PL_COORDINATE_SYSTEM:
			tmp->setCoordinateSystem(int(x), int(y), int(z));
			break;
	}
}

void plVehiclefv(Plenum param, Plfloat* values, Plsizei length) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	btRaycastVehicle* tmp = ctx->getVehicle(ctx->cur_veh);
	if (tmp == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plVehiclefv(): vehicle not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_ADD_WHEEL) {
		if (length != 12) {
			REPORT_ERROR(PL_INVALID_VALUE);
			return;
		}
		tmp->addWheel(
			btVector3(values[0], values[1], values[2]),
			btVector3(values[3], values[4], values[5]),
			btVector3(values[6], values[7], values[8]),
			values[9],
			values[10],
			ctx->tuning,
			bool(values[11]));
	}
}

void plGetWheelfv(Plint wheel, Plenum param, Plfloat* values, Plsizei length) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	btRaycastVehicle* tmp = ctx->getVehicle(ctx->cur_veh);
	if (tmp == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetWheelfv(): vehicle not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_WHEEL_TRANSFORM) {
		if (length != 16) {
			REPORT_ERROR(PL_INVALID_VALUE);
			return;
		}
		tmp->updateWheelTransform(wheel, false);
		tmp->getWheelInfo(wheel).m_worldTransform.getOpenGLMatrix(values);
	} else if (param == PL_WHEEL_TRANSFORM_INTERPOLATION) {
		if (length != 16) {
			REPORT_ERROR(PL_INVALID_VALUE);
			return;
		}
		tmp->updateWheelTransform(wheel, true);
		tmp->getWheelInfo(wheel).m_worldTransform.getOpenGLMatrix(values);
	}
}
/*
		Functions to handle Kinematic Characters
*/
Pluint plGenCharacter() {
	if (!hasContext() || ctx->has_global_error) {
		return 0;
	}
	if (ctx->veh_queue) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGenCharacter(): you can't create a character.\n call btCreate() for make the current character.");
		}
		REPORT_ERROR(PL_INVALID_OPERATION);
		return 0;
	}
	return ctx->queueCharacter();
}

void plBindCharacter(Pluint indx) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (ctx->cur_char <= 0 || indx == 0) {
		ctx->cur_char = indx;
	} else {
		if (ctx->debug_mode) {
			ctx->debug->error("plBindCharacter(): you can't bind this character. unbind the previous character");
		}
		REPORT_ERROR(PL_INVALID_OPERATION);
	}
}

void plDeleteCharacter(Pluint indx) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	ctx->deleteCharacter(indx);
}


void plCharacterf(Plenum param, Plfloat val) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (ctx->char_queue) {
		switch(param) {
			case PL_CHAR_STEP_HEIGHT:
				ctx->char_queue->step_height = val;
				break;
			case PL_CHAR_SPHERE:
				ctx->char_queue->shape = new btSphereShape(btScalar(val));
				break;
		}
		return;
	}
	Character* ch = ctx->getCharacter(ctx->cur_char);
	if (ch == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plCharacterf(): character not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	switch(param) {
		case PL_CHAR_STEP_HEIGHT:
			ch->control->setStepHeight(val);
				break;
		case PL_CHAR_ANGULAR_DAMPING:
			ch->control->setAngularDamping(val);
			break;
		case PL_CHAR_LINEAR_DAMPING:
			ch->control->setLinearDamping(val);
			break;
		case PL_CHAR_PRE_STEP:
			ch->control->preStep(ctx->getWorld());
			break;
		case PL_CHAR_PLAYER_STEP:
			ch->control->playerStep(ctx->getWorld(), val);
			break;
		case PL_CHAR_JUMP_SPEED:
			ch->control->setJumpSpeed(val);
			break;
		case PL_CHAR_FALL_SPEED:
			ch->control->setFallSpeed(val);
			break;
		case PL_CHAR_MAX_JUMP_HEIGHT:
			ch->control->setMaxJumpHeight(val);
			break;
		case PL_CHAR_JUMP:
			ch->control->jump();
			break;
		case PL_CHAR_MAX_SLOPE:
			ch->control->setMaxSlope(val);
			break;
		case PL_CHAR_MAX_PENETRATION_DEPTH:
			ch->control->setMaxPenetrationDepth(val);
			break;
	}
}

void plCharacter3f(Plenum param, Plfloat x, Plfloat y, Plfloat z) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	if (ctx->char_queue) {
		switch(param) {
			case PL_CHAR_UPVEC:
				ctx->char_queue->up.setValue(btScalar(x),btScalar(y),btScalar(z));
				break;
			case PL_CHAR_CAPSULE_X:
				ctx->char_queue->shape = new btCapsuleShapeX(btScalar(x),btScalar(y));
				break;
			case PL_CHAR_CAPSULE_Y:
				ctx->char_queue->shape = new btCapsuleShape(btScalar(x),btScalar(y));
				break;
			case PL_CHAR_CAPSULE_Z:
				ctx->char_queue->shape = new btCapsuleShapeZ(btScalar(x),btScalar(y));
				break;
			case PL_CHAR_BOX:
				ctx->char_queue->shape = new btBoxShape(btVector3(btScalar(x),btScalar(y),btScalar(z)));
				break;
		}
		return;
	}
	Character* ch = ctx->getCharacter(ctx->cur_char);
	if (ch == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plCharacter3f(): character not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	switch(param) {
		case PL_CHAR_WALK_DIRECTION:
			ch->control->setWalkDirection(btVector3(btScalar(x),btScalar(y),btScalar(z)));
			break;
		case PL_CHAR_JUMP:
			ch->control->jump(btVector3(btScalar(x),btScalar(y),btScalar(z)));
			break;
		case PL_CHAR_APPLY_IMPULSE:
			ch->control->applyImpulse(btVector3(btScalar(x),btScalar(y),btScalar(z)));
			break;
		case PL_CHAR_ANGULAR_VELOCITY:
			ch->control->setAngularVelocity(btVector3(btScalar(x),btScalar(y),btScalar(z)));
			break;
		case PL_CHAR_LINEAR_VELOCITY:
			ch->control->setLinearVelocity(btVector3(btScalar(x),btScalar(y),btScalar(z)));
			break;
		case PL_GRAVITY:
			ch->control->setGravity(btVector3(btScalar(x),btScalar(y),btScalar(z)));
			break;
	}
}

void plCharacterfv(Plenum param, Plfloat* values) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	Character* ch = ctx->getCharacter(ctx->cur_char);
	if (ch == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plCharacterfv(): character not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	if (param == PL_TRANSFORM) {
		btTransform t;
		t.setFromOpenGLMatrix(values);
		ch->ghost->setWorldTransform(t);
	} else if (param == PL_CHAR_WALK_DIRECTION) {
		ch->control->setWalkDirection(btVector3(values[0],values[1],values[2]));
	}
}
void plGetCharacterf(Pluint indx, Plenum param, Plfloat* value) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	Character* ch = ctx->getCharacter(indx);
	if (ch == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetCharacterf(): character not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return;
	}
	switch(param) {
		case PL_CHAR_FALL_SPEED:
			*value = ch->control->getFallSpeed();
			break;
		case PL_CHAR_JUMP_SPEED:
			*value = ch->control->getJumpSpeed();
			break;
		case PL_CHAR_ANGULAR_DAMPING:
			*value = ch->control->getAngularDamping();
			break;
		case PL_CHAR_LINEAR_DAMPING:
			*value = ch->control->getLinearDamping();
			break;
	}
}

PLbool plGetCharacterb(Pluint indx, Plenum param) {
	if (!hasContext() || ctx->has_global_error) {
		return false;
	}
	Character* ch = ctx->getCharacter(indx);
	if (ch == NULL) {
		if (ctx->debug_mode) {
			ctx->debug->error("plGetCharacterb(): character not exist");
		}
		REPORT_ERROR(PL_INVALID_NAME);
		return false;
	}
	switch(param) {
		case PL_CHAR_CAN_JUMP:
			return ch->control->canJump();
		case PL_CHAR_ON_GROUND:
			return ch->control->onGround();
	}
	return false;
}

void plGetCharacterfv(Pluint indx, Plenum param, Plfloat* values, Plsizei length) {
	if (!hasContext() || ctx->has_global_error) {
		return;
	}
	Character* ch = ctx->getCharacter(indx);
	if (!ch) {
		REPORT_ERROR(PL_INVALID_NAME);
		if (ctx->debug_mode) {
			ctx->debug->error("plGetCharacterfv(): invalid name");
		}
		return;
	}
	if (param == PL_TRANSFORM) {
		if (length != 16) {
			return;
		}
		btTransform t = ch->ghost->getWorldTransform();
		t.getOpenGLMatrix(values);
	} else if (param == PL_CHAR_LINEAR_VELOCITY) {
		if (length != 3) {
			return;
		}
		btVector3 v = ch->control->getLinearVelocity();
		values[0] = v.getX();
		values[1] = v.getY();
		values[2] = v.getZ();
	} else if (param == PL_CHAR_ANGULAR_VELOCITY) {
		if (length != 3) {
			return;
		}
		btVector3 v = ch->control->getAngularVelocity();
		values[0] = v.getX();
		values[1] = v.getY();
		values[2] = v.getZ();
	}
}
