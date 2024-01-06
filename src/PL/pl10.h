#ifndef _PL10_H_
#define _PL10_H_

#if defined(_WIN32) || defined(__CYGWIN__)
#ifndef PL_BUILD_SHARED_LIB
#define PL_API
#else
#ifdef PL_BUILD_DLL
#define PL_API __declspec(dllexport)
#else
#define PL_API __declspec(dllimport)
#endif
#endif
#else
#if __GNUC__ >= 4
#define PL_API __attribute__((visibility("default")))
#else
#define PL_API
#endif
#endif

typedef unsigned int Pluint;
typedef int Plint;
typedef float Plfloat;
typedef int Plsizei;
typedef int Plenum;
typedef char PLbool;

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*obtContactCallBack)(int user_ptr0,bool match0,int user_ptr1,bool match1);

/* bullet define constants */
#define PL_NONE  							0x000
#define PL_MAX_RAY_RESULTS 					0x00A  /* 10 ray tests */ 

/* bullet error type */
#define PL_NO_ERROR 	 					0x0E0
#define PL_INVALID_NAME	 					0x0E1
#define PL_INVALID_ENUM 	 				0x0E2
#define PL_INVALID_VALUE	 				0x0E3
#define PL_INVALID_OPERATION 	 			0x0E4

/* bullet constant string */
#define PL_VERSION 	 						0x011
#define PL_VENDOR	 						0x012
#define PL_VBASED 	 						0x013
#define PL_DEBUG_INFO 	 					0x014

/* bullet dynamic parameter */
#define PL_GRAVITY 	 						0x0D1
#define PL_ADD_RIGID_BODY 	 				0x0D2
#define PL_REMOVE_RIGID_BODY 	 			0x0D3
#define PL_NUM_RIGID_BODY 	 				0x0D4
#define PL_RAY_CLOSEST_TEST 				0x0D5
#define PL_RAY_ALL_TEST 					0x0D6
#define PL_NUM_RAY_RESULT 					0x0D7
#define PL_RAY_BODY_RESULT 					0x0D8
#define PL_RAY_POINT_RESULT 				0x0D9
#define PL_RAY_NORMAL_RESULT 				0x0DA
#define PL_CLEAR_RAY_RESULTS				0x0DB
#define PL_USE_AXIS_SWEEP3 					0x0DC
#define PL_DEBUG_MODE 						0x0DD
#define PL_ADD_FLAG_RAY_TEST 				0x0DE
#define PL_CONTACT_TEST 					0x0DF
#define PL_FIND_RESULT 						0x0F0
#define PL_INIT_VEHICLE_RAYCASTER			0x0F1
#define PL_INIT_CHARACTER 					0x0F2
#define PL_USE_DBVT 						0x0F3
#define PL_ADD_CHARACTER 					0x0F4
#define PL_ADD_VEHICLE 						0x0F5

/* ray test callback flags */
#define PL_RF_FLT_BACK_FACES 				0x001
#define PL_RF_KP_UNFLIPPED_NORM 			0x002
#define PL_RF_SS_CCR 		 				0x004
#define PL_RF_GJK_CCR 						0x008

/* user pointers */
#define PL_USER_POINTER 					0x050
#define PL_USER_INDEX 						0x051

/* bullet types */
#define PL_DYNAMICS_WORLD	 				0x0A0
#define PL_RIGID_BODY	 					0x0A1
#define PL_COLLISION_SHAPE	 				0x0A2
#define PL_TYPED_CONSTRAINT 				0x0A3
#define PL_RAYCAST_VEHICLE 					0x0A4
#define PL_CHARACTER 						0x0A5

/* bullet shape types (supported by open bt) */
#define PL_BOX_SHAPE	 					0x0C1
#define PL_SPHERE_SHAPE	 					0x0C2
#define PL_CAPSULE_SHAPE	 				0x0C3
#define PL_CYLINDER_SHAPE	 				0x0C4
#define PL_CONE_SHAPE	 					0x0C5
#define PL_STATIC_PLANE_SHAPE	 			0x0C6
#define PL_COMPOUND_SHAPE 					0x0C7
#define PL_CYLINDER_SHAPE_X 				0x0C8
#define PL_CYLINDER_SHAPE_Z 				0x0C9
#define PL_CAPSULE_SHAPE_X 					0x0CA
#define PL_CAPSULE_SHAPE_Z 					0x0CB
#define PL_CONE_SHAPE_X 					0x0CC
#define PL_CONE_SHAPE_Z 					0x0CD
#define PL_BVH_TRIANGLE_MESH_SHAPE 			0x0CE
#define PL_CONVEX_HULL_SHAPE 				0x0CF

/* work shape buffers */
#define PL_VERTEX_BUFFER 					0x0F0
#define PL_INDEX_BUFFER 					0x0F1

/* bullet shape input and output parameter */
#define PL_SHAPE_TYPE 						0xAC1
#define PL_EXTENT 							0xAC2
#define PL_RADIUS 							0xAC3
#define PL_HEIGHT 							0xAC4
#define PL_PLANE_NORMAL 					0xAC5
#define PL_PLANE_CONSTANT					0xAC6
#define PL_ADD_CHILD_SHAPE 					0xAC7
#define PL_REMOVE_CHILD_SHAPE 				0xAC8
#define PL_LOCAL_SCALING 					0xAC9
#define PL_OPTIMIZE_CONVEX_HULL 			0xACA
#define PL_NUM_CHILD_SHAPES 				0xACB

/* bullet rigidbody and collision object paramter */
#define PL_MASS	 							0xCD1
#define PL_POSITION	 						0xCD2
#define PL_ORIENTATION	 					0xCD3
#define PL_BASIS	 						0xCD4
#define PL_TRANSFORM	 					0xCD5
#define PL_LOCAL_INERTIA 	 				0xCD6
#define PL_DAMPING 							0xCD7
#define PL_LINEAR_DAMPING 					0xCD8
#define PL_ANGULAR_DAMPING 					0xCD9
#define PL_LINEAR_FACTOR 					0xCDA
#define PL_ANGULAR_FACTOR 					0xCDB
#define PL_APPLY_CENTRAL_FORCE 				0xCDC
#define PL_APPLY_FORCE 						0xCDE
#define PL_APPLY_TORQUE 					0xCDD
#define PL_APPLY_IMPULSE 					0xCDF
#define PL_APPLY_CENTRAL_IMPULSE 			0xCF0
#define PL_APPLY_TORQUE_IMPULSE 			0xCF1
#define PL_IS_IN_WORLD 						0xCF2
#define PL_TOTAL_FORCE 						0xCF3
#define PL_TOTAL_TORQUE						0xCF4
#define PL_LINEAR_VELOCITY 					0xCF5
#define PL_ANGULAR_VELOCITY					0xCF6
#define PL_ROLLING_FRICTION 				0xCF7
#define PL_SPINNING_FRICTION 				0xCF8
#define PL_FRICTION 						0xCF9
#define PL_ANISOTROPIC_FRICTION 			0xCFA
#define PL_ANISOTROPIC_ROLLING_FRICTION 	0xCFB
#define PL_DIS_DACTIVATION					0xCFC
#define PL_DIS_SIMULATION 					0xCFD
#define PL_ACTIVATE 						0xCFE
#define PL_ISLAND_SLEEPING 					0xFCE
#define PL_CF_KINEMATIC_OBJECT 				0xCFF
#define PL_CF_STATIC_OBJECT 				0xD00
#define PL_CF_CUSTOM_MATERIAL_CALLBACK 		0xD01
#define PL_ACTIVATION_STATE  				0xD02
#define PL_ADD_COLLISION_FLAG 				0xD03
#define PL_COLLISION_CALLBACK_FILTER 		0xD04
#define PL_COLLISION_CALLBACK_FLAG 			0xD05
#define PL_RESTITUTION 						0xD06
#define PL_COL_WORLD_TRANSFORM 				0xD07
#define PL_COL_INTERP_WORLD_TRANSFORM		0xD08
#define PL_COL_INTERP_LINEAR_VEL 			0xD09
#define PL_COL_INTERP_ANGULAR_VEL			0xD0A

/* bullet motion state parameter */
#define PL_MOTION_STATE_TRANSFORM 			0xCA0
#define PL_MOTION_STATE_POSITION			0xCA1

/* type of constraints */
#define PL_POINT2PONT_CONSTRAINT 			0xED0
#define PL_SLIDER_CONSTRAINT 				0xED1
#define PL_HINGE_CONSTRAINT 				0xED2
#define PL_HINGE2_CONSTRAINT 				0xED3
#define PL_G6DOF_CONSTRAINT 				0xED4
#define PL_G6DOF_SPRING_CONSTRAINT 			0xED5
#define PL_G6DOF_SPRING2_CONSTRAINT 		0xED6
#define PL_CONE_TWIST_CONSTRAINT 			0xED7
#define PL_ADD_CONSTRAINT 					0xED8
#define PL_REMOVE_CONSTRAINT 				0xED9

/* Contructor Parameters */
#define PL_CONSTR_RBODY_A 					0xEF0
#define PL_CONSTR_RBODY_B 					0xEF1
#define PL_CONSTR_PIVOT_A 					0xEF2
#define PL_CONSTR_PIVOT_B 					0xEF3
#define PL_CONSTR_FRAME_A					0xEF4
#define PL_CONSTR_FRAME_B					0xEF5
#define PL_CONSTR_IN_AXIS_A					0xEF6
#define PL_CONSTR_IN_AXIS_B					0xEF7
#define PL_CONSTR_USE_REFERENCE_A 			0xEF8
/* Hinge Constraint */
#define PL_HC_ENABLE_ANGULAR_MOTOR 			0xBC0
#define PL_HC_MAX_MOTOR_IMPULSE 			0xBC1
#define PL_HC_TARGET_VELOCITY 				0xBC2
#define PL_HC_ANGULAR_ONLY					0xBC3
#define PL_HC_LIMITS						0xBC4
/* Generic 2 Dof*/
#define PL_G6D_LOWER_LIN_LIMIT				0xBC6
#define PL_G6D_UPPER_LIN_LIMIT				0xBC7
#define PL_G6D_LOWER_ANG_LIMIT				0xBC8
#define PL_G6D_UPPER_ANG_LIMIT				0xBC9
#define PL_G6D_LIMIT_LIN_X					0xBCA
#define PL_G6D_LIMIT_LIN_Y					0xBCB
#define PL_G6D_LIMIT_LIN_Z					0xBCC
#define PL_G6D_LIMIT_ANG_X					0xBCD
#define PL_G6D_LIMIT_ANG_Y					0xBCE
#define PL_G6D_LIMIT_ANG_Z					0xBCF
/*  Generic 2 dof spring*/
#define PL_G6DS_ENABLE_SPRING 				0xBD0
#define PL_G6DS_DISABLE_SPRING 				0xBD1
#define PL_G6DS_STIFFNESS_SPRING 			0xBD2
#define PL_G6DS_DAMPING_SPRING 				0xBD3
#define PL_G6DS_EQUILI_POINT_SPRING 		0xBD4
/*  Generic 2 dof spring 2*/
#define PL_G6DS2_LOWER_LIN_LIMIT			0xBD5
#define PL_G6DS2_UPPER_LIN_LIMIT			0xBD6
#define PL_G6DS2_LOWER_ANG_LIMIT			0xBD7
#define PL_G6DS2_UPPER_ANG_LIMIT			0xBD8
#define PL_G6DS2_ENABLE_SPRING 				0xBD9
#define PL_G6DS2_DISABLE_SPRING 			0xBDA
#define PL_G6DS2_STIFFNESS_SPRING 			0xBDB
#define PL_G6DS2_DAMPING_SPRING 			0xBDC
#define PL_G6DS2_EQUILI_POINT_SPRING 		0xBDD
#define PL_G6DS2_ENABLE_MOTOR				0xBDE
#define PL_G6DS2_SET_SERVO 					0xBDF
#define PL_G6DS2_TARGET_VELOCITY 			0xBF0
#define PL_G6DS2_SERVO_TARGET 				0xBF1
#define PL_G6DS2_MAX_MOTOR_FORCE 			0xBF2
#define PL_G6DS2_BOUNCE						0xBF3
/* point 2 point */
#define PL_P2P_PIVOT_A 						0xBF4
#define PL_P2P_PIVOT_B 						0xBF5
#define PL_P2P_UPDATE_RHS 					0xBF6
/* slider */	
#define PL_SLDR_SOFTNESS_DIR_LINEAR 		0xBF7
#define PL_SLDR_SOFTNESS_DIR_ANGULAR 		0xBF8
#define PL_SLDR_SOFTNESS_LIMIT_LIN 			0xBF9
#define PL_SLDR_SOFTNESS_LIMIT_ANG			0xBFA
#define PL_SLDR_SOFTNESS_ORTHO_LIN 			0xBFB
#define PL_SLDR_SOFTNESS_ORTHO_ANG			0xBFC
#define PL_SLDR_DAMPING_DIR_LINEAR 			0xBFD
#define PL_SLDR_DAMPING_DIR_ANGULAR 		0xBFE
#define PL_SLDR_DAMPING_LIMIT_LIN 			0xBFF
#define PL_SLDR_DAMPING_LIMIT_ANG			0xC00
#define PL_SLDR_DAMPING_ORTHO_LIN 			0xC01
#define PL_SLDR_DAMPING_ORTHO_ANG			0xC02
#define PL_SLDR_RESTITUTION_DIR_LINEAR 		0xC03
#define PL_SLDR_RESTITUTION_DIR_ANGULAR 	0xC04
#define PL_SLDR_RESTITUTION_LIMIT_LIN 		0xC05
#define PL_SLDR_RESTITUTION_LIMIT_ANG		0xC06
#define PL_SLDR_RESTITUTION_ORTHO_LIN 		0xC07
#define PL_SLDR_RESTITUTION_ORTHO_ANG		0xC08
#define PL_SLDR_POWERED_LINEAR_MOTOR 		0xC09
#define PL_SLDR_POWERED_ANGULAR_MOTOR 		0xC0A
#define PL_SLDR_MAX_ANG_MOTOR_FORCE 		0xC0B
#define PL_SLDR_MAX_LIN_MOTOR_FORCE 		0xC0C
#define PL_SLDR_TARGET_LIN_MOTOR_VEL 		0xC0D
#define PL_SLDR_TARGET_ANG_MOTOR_VEL 		0xC0E
/* hinge 2 */
#define PL_HC2_LIMITS						0xC0F
#define PL_HC2_LOWER_LIMIT					0xC10
#define PL_HC2_UPPER_LIMIT					0xC11
/* cone twist */
#define PL_CTWIST_LIMITS					0xACC

/* vehicule ray casting (vehicule suspension simulation) */
#define PL_VEHICLE_CHASSIS 					0xC12
#define PL_SUSPENSION_STIFFNESS 			0xC13
#define PL_SUSPENSION_DAMPING 				0xC14
#define PL_SUSPENSION_COMPRESION			0xC15
#define PL_MAX_SUSPENSION_TRAVEL			0xC16
#define PL_FRICTION_SLIP					0xC17
#define PL_MAX_SUSPENSION_FORCE				0xC18
#define PL_COORDINATE_SYSTEM				0xC19

/* 			float Data Offsets (Add Wheel)
*****************************************
	[0,2] = connection point xyz
	[3,5] = wheel direction xyz
	[6,8] = wheel axis  xyz
	[9] = suspension rest lenght
	[10] = wheel radius
	[11] = bool [0,1] isFrontWheel
	Total lenght = 12
*/

#define PL_ADD_WHEEL 						0xC1A

#define PL_WHEEL_ENGINE_FORCE 				0xC1B
#define PL_WHEEL_STEERING 					0xC1C
#define PL_WHEEL_BRAKE 						0xC1D
#define PL_WHEEL_ROLL_INFLUENCE 			0xC1E
#define PL_WHEEL_TRANSFORM 					0xC1F
#define PL_WHEEL_TRANSFORM_INTERPOLATION 	0xC20

/* kinematic character */
#define PL_CHAR_STEP_HEIGHT 				0xC21
#define PL_CHAR_CAPSULE_X 					0xC22
#define PL_CHAR_CAPSULE_Y 					0xC23
#define PL_CHAR_CAPSULE_Z					0xC24
#define PL_CHAR_BOX 						0xC25
#define PL_CHAR_UPVEC						0xC26
#define PL_CHAR_SPHERE 						0xC27
#define PL_CHAR_WALK_DIRECTION 				0xC28
#define PL_CHAR_ANGULAR_VELOCITY 			0xC29
#define PL_CHAR_LINEAR_VELOCITY 			0xC2A
#define PL_CHAR_ANGULAR_DAMPING 			0xC2B
#define PL_CHAR_LINEAR_DAMPING				0xC2C
#define PL_CHAR_PRE_STEP 					0xC2D
#define PL_CHAR_PLAYER_STEP 				0xC2E
#define PL_CHAR_WRAP 						0xC2F
#define PL_CHAR_JUMP_SPEED 					0xC30
#define PL_CHAR_FALL_SPEED 					0xC31
#define PL_CHAR_MAX_JUMP_HEIGHT 			0xC32
#define PL_CHAR_JUMP 						0xC33
#define PL_CHAR_APPLY_IMPULSE 				0xC34
#define PL_CHAR_MAX_PENETRATION_DEPTH 		0xC35
#define PL_CHAR_MAX_SLOPE 					0xC36
#define PL_CHAR_CAN_JUMP 					0xC37
#define PL_CHAR_ON_GROUND 					0xC38

/* core Functions */
PL_API PLbool plCreateContext();
PL_API void plDestroyContext();
PL_API Plint plGetError();
PL_API const char* plGetString(Plenum param);
PL_API Plint plGetInteger(Plenum param);
PL_API void plCreate(Plenum target);
PL_API void plStepSimulation(Plfloat timeStep,Plsizei maxSubSteps,Plfloat fixedTimeStep);

/* World Functions */
PL_API void plDynamicWorldi(Plenum param,Plint value);
PL_API void plDynamicWorld3f(Plenum param,Plfloat v1,Plfloat v2,Plfloat v3);
PL_API void plDynamicWorldfv(Plenum param,Plfloat* values,Plsizei lenght);

PL_API void plGetDynamicWorldi(Plenum param,Plint* value);
PL_API void plGetDynamicWorld3f(Plenum param,Plfloat* v1,Plfloat* v2,Plfloat* v3);
PL_API void plGetDynamicWorldiv(Plenum param,Plint* values,Plsizei lenght);
PL_API void plGetDynamicWorldfv(Plenum param,Plfloat* values,Plsizei lenght);
PL_API void plContactCallBack(obtContactCallBack callback);

/* Body Functions*/
PL_API Pluint plGenBody();
PL_API void plDeleteBody(Pluint body);
PL_API void plBindBody(Pluint body);
PL_API void plRigidBodyi(Plenum param,Plint value);
PL_API void plRigidBodyf(Plenum param,Plfloat value);
PL_API void plRigidBody3f(Plenum param,Plfloat v1,Plfloat v2,Plfloat v3);
PL_API void plRigidBodyfv(Plenum param,Plfloat* values,Plsizei lenght);

PL_API void plGetRigidBodyi(Pluint body,Plenum param,Plint* value);
PL_API void plGetRigidBodyf(Pluint body,Plenum param,Plfloat* value);
PL_API void plGetRigidBody3f(Pluint body,Plenum param,Plfloat* v1,Plfloat* v2,Plfloat* v3);
PL_API void plGetRigidBodyfv(Pluint body,Plenum param,Plfloat* values,Plint lenght);

/* Shape Functions */
PL_API Pluint plGenShape();
PL_API void plBindShape(Pluint shape);
PL_API void plShapei(Plenum param,Plint value);
PL_API void plShapef(Plenum param,Plfloat value);
PL_API void plShape3f(Plenum param,Plfloat v1,Plfloat v2,Plfloat v3);
PL_API void plShapefv(Plenum param,Plfloat* values,Plsizei lenght);
PL_API void plBufferData(Plenum type,Plsizei size, void* data);

PL_API void plGetShapei(Pluint shape,Plenum param,Plint* value);
PL_API void plGetShapefv(Pluint shape,Plenum param,Plfloat* values,Plsizei lenght);

/* Constraint Capability */
PL_API Pluint plGenConstraint(Plenum type);
PL_API void plBindConstraint(Pluint ctr);
PL_API void plDeleteConstraint(Pluint ctr);
PL_API void plConstrainti(Plenum param,Plint value);
PL_API void plConstraintf(Plenum param,Plfloat value);
PL_API void plConstraint3f(Plenum param,Plfloat x,Plfloat y,Plfloat z);
PL_API void plConstraintfv(Plenum param,Plfloat* values,Plsizei lenght);

/* Vehicle Ray Casting Extension */
PL_API Pluint plGenVehicle();
PL_API void plBindVehicle(Pluint indx);
PL_API void plDeleteVehicle(Pluint indx);
PL_API void plWheelf(Plint wheel,Plenum param,Plfloat value);
PL_API void plVehiclei(Plenum param,Plint value);
PL_API void plVehiclef(Plenum param,Plfloat value);
PL_API void plVehicle3f(Plenum param,Plfloat x,Plfloat y,Plfloat z);
PL_API void plVehiclefv(Plenum param,Plfloat* values,Plsizei lenght);
PL_API void plGetWheelfv(Plint wheel,Plenum param,Plfloat* values,Plsizei lenght);

/* Character extension */
PL_API Pluint plGenCharacter();
PL_API void plBindCharacter(Pluint indx);
PL_API void plDeleteCharacter(Pluint indx);
PL_API void plCharacterf(Plenum param,Plfloat val);
PL_API void plCharacter3f(Plenum param,Plfloat x,Plfloat y,Plfloat z);
PL_API void plCharacterfv(Plenum param,Plfloat* values);
PL_API void plGetCharacterf(Pluint indx,Plenum param,Plfloat* value);
PL_API PLbool plGetCharacterb(Pluint indx,Plenum param);
PL_API void plGetCharacterfv(Pluint indx,Plenum param,Plfloat* values,Plsizei lenght);

#ifdef __cplusplus
}
#endif

#endif
