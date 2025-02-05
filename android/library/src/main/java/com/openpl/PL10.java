package com.openpl;
import java.nio.*;

public class PL10
{
	/* bullet define constants */
	public static final int PL_NONE  						=	0x000;
	public static final int PL_MAX_RAY_RESULTS 				=	0x008; /* 8 ray tests */

	/* bullet boolean values */
	public static final int PL_TRUE 						=	0x001;
	public static final int PL_FALSE  						=	0x000;

	/* bullet error type */
	public static final int PL_NO_ERROR 	 				=	0x0E0;
	public static final int PL_INVALID_NAME	 				=	0x0E1;
	public static final int PL_INVALID_ENUM 	 			=	0x0E2;
	public static final int PL_INVALID_VALUE	 			=	0x0E3;
	public static final int PL_INVALID_OPERATION 	 		=	0x0E4;

	/* bullet constant string */
	public static final int PL_VERSION 	 					=	0x011;
	public static final int PL_VENDOR	 					=	0x012;
	public static final int PL_VBASED 	 					=	0x013;
	public static final int PL_DEBUG_INFO 	 				=	0x014;

	/* bullet dynamic parameter */
	public static final int PL_GRAVITY 	 					=	0x0D1;
	public static final int PL_ADD_RIGID_BODY 	 			=	0x0D2;
	public static final int PL_REMOVE_RIGID_BODY 	 		=	0x0D3;
	public static final int PL_NUM_RIGID_BODY 	 			=	0x0D4;
	public static final int PL_RAY_CLOSEST_TEST 			=	0x0D5;
	public static final int PL_RAY_ALL_TEST 				=	0x0D6;
	public static final int PL_NUM_RAY_RESULT 				=	0x0D7;
	public static final int PL_RAY_BODY_RESULT 				=	0x0D8;
	public static final int PL_RAY_POINT_RESULT 			=	0x0D9;
	public static final int PL_RAY_NORMAL_RESULT 			=	0x0DA;
	public static final int PL_CLEAR_RAY_RESULTS			=	0x0DB;
	public static final int PL_USE_AXIS_SWEEP3 				=	0x0DC;
	public static final int PL_DEBUG_MODE 					=	0x0DD;
	public static final int PL_ADD_FLAG_RAY_TEST 			=	0x0DE;
	public static final int PL_CONTACT_TEST 				=	0x0DF;
	public static final int PL_FIND_RESULT 					=	0x0F0;
	public static final int PL_INIT_VEHICLE_RAYCASTER		=	0x0F1;
	public static final int PL_INIT_CHARACTER 				=	0x0F2;
	public static final int PL_USE_DBVT 					=	0x0F3;
	public static final int PL_ADD_CHARACTER 				=	0x0F4;
	public static final int PL_ADD_VEHICLE 					=	0x0F5;
	
	/* ray test callback flags */
	public static final int PL_RF_FLT_BACK_FACES 			=	0x001;
	public static final int PL_RF_KP_UNFLIPPED_NORM 		=	0x002;
	public static final int PL_RF_SS_CCR 		 			=	0x004;
	public static final int PL_RF_GJK_CCR 					=	0x008;

	/* user pointers */
	public static final int PL_USER_POINTER 				=	0x050;
	public static final int PL_USER_INDEX 					=	0x051;

	/* bullet types */
	public static final int PL_DYNAMICS_WORLD	 			=	0x0A0;
	public static final int PL_RIGID_BODY	 				=	0x0A1;
	public static final int PL_COLLISION_SHAPE	 			=	0x0A2;
	public static final int PL_TYPED_CONSTRAINT 			=	0x0A3;
	public static final int PL_RAYCAST_VEHICLE 				=	0x0A4;
	public static final int PL_CHARACTER 					=	0x0A5;

	/* bullet shape types (supported by open pl) */
	public static final int PL_BOX_SHAPE	 				=	0x0C1;
	public static final int PL_SPHERE_SHAPE	 				=	0x0C2;
	public static final int PL_CAPSULE_SHAPE	 			=	0x0C3;
	public static final int PL_CYLINDER_SHAPE	 			=	0x0C4;
	public static final int PL_CONE_SHAPE	 				=	0x0C5;
	public static final int PL_STATIC_PLANE_SHAPE	 		=	0x0C6;
	public static final int PL_COMPOUND_SHAPE 				=	0x0C7;
	public static final int PL_CYLINDER_SHAPE_X 			=	0x0C8;
	public static final int PL_CYLINDER_SHAPE_Z 			=	0x0C9;
	public static final int PL_CAPSULE_SHAPE_X 				=	0x0CA;
	public static final int PL_CAPSULE_SHAPE_Z 				=	0x0CB;
	public static final int PL_CONE_SHAPE_X 				=	0x0CC;
	public static final int PL_CONE_SHAPE_Z 				=	0x0CD;
	public static final int PL_BVH_TRIANGLE_MESH_SHAPE 		=	0x0CE;
	public static final int PL_CONVEX_HULL_SHAPE 			=	0x0CF;

	/* work shape buffers */
	public static final int PL_VERTEX_BUFFER 				=	0x0F0;
	public static final int PL_INDEX_BUFFER 				=	0x0F1;

	/* bullet shape input and output parameter */
	public static final int PL_SHAPE_TYPE 					=	0xAC1;
	public static final int PL_EXTENT 						=	0xAC2;
	public static final int PL_RADIUS 						=	0xAC3;
	public static final int PL_HEIGHT 						=	0xAC4;
	public static final int PL_PLANE_NORMAL 				=	0xAC5;
	public static final int PL_PLANE_CONSTANT				=	0xAC6;
	public static final int PL_ADD_CHILD_SHAPE 				=	0xAC7;
	public static final int PL_REMOVE_CHILD_SHAPE 			=	0xAC8;
	public static final int PL_LOCAL_SCALING 				=	0xAC9;
	public static final int PL_OPTIMIZE_CONVEX_HULL 		=	0xACA;
	public static final int PL_NUM_CHILD_SHAPES 			=	0xACB;

	/* bullet rigidbody and collision object paramter */
	public static final int PL_MASS	 						=	0xCD1;
	public static final int PL_POSITION	 					=	0xCD2;
	public static final int PL_ORIENTATION	 				=	0xCD3;
	public static final int PL_BASIS	 					=	0xCD4;
	public static final int PL_TRANSFORM	 				=	0xCD5;
	public static final int PL_LOCAL_INERTIA 	 			=	0xCD6;
	public static final int PL_DAMPING 						=	0xCD7;
	public static final int PL_LINEAR_DAMPING 				=	0xCD8;
	public static final int PL_ANGULAR_DAMPING 				=	0xCD9;
	public static final int PL_LINEAR_FACTOR 				=	0xCDA;
	public static final int PL_ANGULAR_FACTOR 				=	0xCDB;
	public static final int PL_APPLY_CENTRAL_FORCE 			=	0xCDC;
	public static final int PL_APPLY_FORCE 					=	0xCDE;
	public static final int PL_APPLY_TORQUE 				=	0xCDD;
	public static final int PL_APPLY_IMPULSE 				=	0xCDF;
	public static final int PL_APPLY_CENTRAL_IMPULSE 		=	0xCF0;
	public static final int PL_APPLY_TORQUE_IMPULSE 		=	0xCF1;
	public static final int PL_IS_IN_WORLD 					=	0xCF2;
	public static final int PL_TOTAL_FORCE 					=	0xCF3;
	public static final int PL_TOTAL_TORQUE					=	0xCF4;
	public static final int PL_LINEAR_VELOCITY 				=	0xCF5;
	public static final int PL_ANGULAR_VELOCITY				=	0xCF6;
	public static final int PL_ROLLING_FRICTION 			=	0xCF7;
	public static final int PL_SPINNING_FRICTION 			=	0xCF8;
	public static final int PL_FRICTION 					=	0xCF9;
	public static final int PL_ANISOTROPIC_FRICTION 		=	0xCFA;
	public static final int PL_ANISOTROPIC_ROLLING_FRICTION = 	0xCFB;
	public static final int PL_DIS_DACTIVATION				=	0xCFC;
	public static final int PL_DIS_SIMULATION 				=	0xCFD;
	public static final int PL_ACTIVATE 					=	0xCFE;
	public static final int PL_ISLAND_SLEEPING 				=	0xFCE;
	public static final int PL_CF_KINEMATIC_OBJECT 			=	0xCFF;
	public static final int PL_CF_STATIC_OBJECT 			=   0xD00;
	public static final int PL_CF_CUSTOM_MATERIAL_CALLBACK 	=	0xD01;
	public static final int PL_ACTIVATION_STATE  			=	0xD02;
	public static final int PL_ADD_COLLISION_FLAG 			=	0xD03;
	public static final int PL_COLLISION_CALLBACK_FILTER 	=	0xD04;
	public static final int PL_COLLISION_CALLBACK_FLAG 		=	0xD05;
	public static final int PL_RESTITUTION 					=	0xD06;
	public static final int PL_COL_WORLD_TRANSFORM 			=	0xD07;
	public static final int PL_COL_INTERP_WORLD_TRANSFORM	=	0xD08;
	public static final int PL_COL_INTERP_LINEAR_VEL 		=	0xD09;
	public static final int PL_COL_INTERP_ANGULAR_VEL		=	0xD0A;

	/* bullet motion state parameter */
	public static final int PL_MOTION_STATE_TRANSFORM 		=	0xCA0;
	public static final int PL_MOTION_STATE_POSITION		=	0xCA1;

	/* type of constraints */
	public static final int PL_POINT2PONT_CONSTRAINT 		=	0xED0;
	public static final int PL_SLIDER_CONSTRAINT 			=	0xED1;
	public static final int PL_HINGE_CONSTRAINT 			=	0xED2;
	public static final int PL_HINGE2_CONSTRAINT 			=	0xED3;
	public static final int PL_G6DOF_CONSTRAINT 			=	0xED4;
	public static final int PL_G6DOF_SPRING_CONSTRAINT 		=	0xED5;
	public static final int PL_G6DOF_SPRING2_CONSTRAINT     =	0xED6;
	public static final int PL_CONE_TWIST_CONSTRAINT 		=	0xED7;
	public static final int PL_ADD_CONSTRAINT 				=	0xED8;
	public static final int PL_REMOVE_CONSTRAINT 			=	0xED9;

	/* Contructor Parameters */
	public static final int PL_CONSTR_RBODY_A 				=	0xEF0;
	public static final int PL_CONSTR_RBODY_B 				=	0xEF1;
	public static final int PL_CONSTR_PIVOT_A 				=	0xEF2;
	public static final int PL_CONSTR_PIVOT_B 				=	0xEF3;
	public static final int PL_CONSTR_FRAME_A				=	0xEF4;
	public static final int PL_CONSTR_FRAME_B				=	0xEF5;
	public static final int PL_CONSTR_IN_AXIS_A				=	0xEF6;
	public static final int PL_CONSTR_IN_AXIS_B				=	0xEF7;
	public static final int PL_CONSTR_USE_REFERENCE_A 		=	0xEF8;
	/* Hinge Constraint */
	public static final int PL_HC_ENABLE_ANGULAR_MOTOR 		=	0xBC0;
	public static final int PL_HC_MAX_MOTOR_IMPULSE 		=	0xBC1;
	public static final int PL_HC_TARGET_VELOCITY 			=	0xBC2;
	public static final int PL_HC_ANGULAR_ONLY				=	0xBC3;
	public static final int PL_HC_LIMITS					=	0xBC4;
	/* Generic 2 Dof*/
	public static final int PL_G6D_LOWER_LIN_LIMIT			=	0xBC6;
	public static final int PL_G6D_UPPER_LIN_LIMIT			=	0xBC7;
	public static final int PL_G6D_LOWER_ANG_LIMIT			=	0xBC8;
	public static final int PL_G6D_UPPER_ANG_LIMIT			=	0xBC9;
	public static final int PL_G6D_LIMIT_LIN_X				=	0xBCA;
	public static final int PL_G6D_LIMIT_LIN_Y				=	0xBCB;
	public static final int PL_G6D_LIMIT_LIN_Z				=	0xBCC;
	public static final int PL_G6D_LIMIT_ANG_X				=	0xBCD;
	public static final int PL_G6D_LIMIT_ANG_Y				=	0xBCE;
	public static final int PL_G6D_LIMIT_ANG_Z				=	0xBCF;
	/*  Generic 2 dof spring*/
	public static final int PL_G6DS_ENABLE_SPRING 			=	0xBD0;
	public static final int PL_G6DS_DISABLE_SPRING 			=	0xBD1;
	public static final int PL_G6DS_STIFFNESS_SPRING 		=	0xBD2;
	public static final int PL_G6DS_DAMPING_SPRING 			=	0xBD3;
	public static final int PL_G6DS_EQUILI_POINT_SPRING 	=	0xBD4;
	/*  Generic 2 dof spring 2*/
	public static final int PL_G6DS2_LOWER_LIN_LIMIT		=	0xBD5;
	public static final int PL_G6DS2_UPPER_LIN_LIMIT		=	0xBD6;
	public static final int PL_G6DS2_LOWER_ANG_LIMIT		=	0xBD7;
	public static final int PL_G6DS2_UPPER_ANG_LIMIT		=	0xBD8;
	public static final int PL_G6DS2_ENABLE_SPRING 			=	0xBD9;
	public static final int PL_G6DS2_DISABLE_SPRING 		=	0xBDA;
	public static final int PL_G6DS2_STIFFNESS_SPRING 		=	0xBDB;
	public static final int PL_G6DS2_DAMPING_SPRING 		=	0xBDC;
	public static final int PL_G6DS2_EQUILI_POINT_SPRING 	=	0xBDD;
	public static final int PL_G6DS2_ENABLE_MOTOR			=	0xBDE;
	public static final int PL_G6DS2_SET_SERVO 				=	0xBDF;
	public static final int PL_G6DS2_TARGET_VELOCITY 		=	0xBF0;
	public static final int PL_G6DS2_SERVO_TARGET 			=	0xBF1;
	public static final int PL_G6DS2_MAX_MOTOR_FORCE 		=	0xBF2;
	public static final int PL_G6DS2_BOUNCE					=	0xBF3;
	/* point 2 point */
	public static final int PL_P2P_PIVOT_A 					=	0xBF4;
	public static final int PL_P2P_PIVOT_B 					=	0xBF5;
	public static final int PL_P2P_UPDATE_RHS 				=	0xBF6;
	/* slider */	;
	public static final int PL_SLDR_SOFTNESS_DIR_LINEAR 	=	0xBF7;
	public static final int PL_SLDR_SOFTNESS_DIR_ANGULAR 	=	0xBF8;
	public static final int PL_SLDR_SOFTNESS_LIMIT_LIN 		=	0xBF9;
	public static final int PL_SLDR_SOFTNESS_LIMIT_ANG		=	0xBFA;
	public static final int PL_SLDR_SOFTNESS_ORTHO_LIN 		=	0xBFB;
	public static final int PL_SLDR_SOFTNESS_ORTHO_ANG		=	0xBFC;
	public static final int PL_SLDR_DAMPING_DIR_LINEAR 		=	0xBFD;
	public static final int PL_SLDR_DAMPING_DIR_ANGULAR 	=	0xBFE;
	public static final int PL_SLDR_DAMPING_LIMIT_LIN 		=	0xBFF;
	public static final int PL_SLDR_DAMPING_LIMIT_ANG		=	0xC00;
	public static final int PL_SLDR_DAMPING_ORTHO_LIN 		=	0xC01;
	public static final int PL_SLDR_DAMPING_ORTHO_ANG		=	0xC02;
	public static final int PL_SLDR_RESTITUTION_DIR_LINEAR 	=	0xC03;
	public static final int PL_SLDR_RESTITUTION_DIR_ANGULAR =	0xC04;
	public static final int PL_SLDR_RESTITUTION_LIMIT_LIN 	=	0xC05;
	public static final int PL_SLDR_RESTITUTION_LIMIT_ANG	=	0xC06;
	public static final int PL_SLDR_RESTITUTION_ORTHO_LIN 	=	0xC07;
	public static final int PL_SLDR_RESTITUTION_ORTHO_ANG	=	0xC08;
	public static final int PL_SLDR_POWERED_LINEAR_MOTOR 	=	0xC09;
	public static final int PL_SLDR_POWERED_ANGULAR_MOTOR 	=	0xC0A;
	public static final int PL_SLDR_MAX_ANG_MOTOR_FORCE 	=	0xC0B;
	public static final int PL_SLDR_MAX_LIN_MOTOR_FORCE 	=	0xC0C;
	public static final int PL_SLDR_TARGET_LIN_MOTOR_VEL 	=	0xC0D;
	public static final int PL_SLDR_TARGET_ANG_MOTOR_VEL 	=	0xC0E;
	/* hinge 2 */
	public static final int PL_HC2_LIMITS					=	0xC0F;
	public static final int PL_HC2_LOWER_LIMIT				=	0xC10;
	public static final int PL_HC2_UPPER_LIMIT				=	0xC11;
	/* cone twist */
	public static final int PL_CTWIST_LIMITS				=	0xACC;

	/* vehicule ray casting (vehicule suspension simulation) */
	public static final int PL_VEHICLE_CHASSIS 				=	0xC12;
	public static final int PL_SUSPENSION_STIFFNESS 		=	0xC13;
	public static final int PL_SUSPENSION_DAMPING 			=	0xC14;
	public static final int PL_SUSPENSION_COMPRESION		=	0xC15;
	public static final int PL_MAX_SUSPENSION_TRAVEL		=	0xC16;
	public static final int PL_FRICTION_SLIP				=	0xC17;
	public static final int PL_MAX_SUSPENSION_FORCE			=	0xC18;
	public static final int PL_COORDINATE_SYSTEM			=	0xC19;
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
	
	public static final int PL_ADD_WHEEL 					=	0xC1A;

	public static final int PL_WHEEL_ENGINE_FORCE 			=	0xC1B;
	public static final int PL_WHEEL_STEERING 				=	0xC1C;
	public static final int PL_WHEEL_BRAKE 					=	0xC1D;
	public static final int PL_WHEEL_ROLL_INFLUENCE 		=	0xC1E;
	public static final int PL_WHEEL_TRANSFORM 				=	0xC1F;
	public static final int PL_WHEEL_TRANSFORM_INTERPOLATION =	0xC20;

	/* kinematic character */
	public static final int PL_CHAR_STEP_HEIGHT 			=	0xC21;
	public static final int PL_CHAR_CAPSULE_X 				=	0xC22;
	public static final int PL_CHAR_CAPSULE_Y 				=	0xC23;
	public static final int PL_CHAR_CAPSULE_Z				=	0xC24;
	public static final int PL_CHAR_BOX 					=	0xC25;
	public static final int PL_CHAR_UPVEC					=	0xC26;
	public static final int PL_CHAR_SPHERE 					=	0xC27;
	public static final int PL_CHAR_WALK_DIRECTION 			=	0xC28;
	public static final int PL_CHAR_ANGULAR_VELOCITY 		=	0xC29;
	public static final int PL_CHAR_LINEAR_VELOCITY 		=	0xC2A;
	public static final int PL_CHAR_ANGULAR_DAMPING 		=	0xC2B;
	public static final int PL_CHAR_LINEAR_DAMPING			=	0xC2C;
	public static final int PL_CHAR_PRE_STEP 				=	0xC2D;
	public static final int PL_CHAR_PLAYER_STEP 			=	0xC2E;
	public static final int PL_CHAR_WRAP 					=	0xC2F;
	public static final int PL_CHAR_JUMP_SPEED 				=	0xC30;
	public static final int PL_CHAR_FALL_SPEED 				=	0xC31;
	public static final int PL_CHAR_MAX_JUMP_HEIGHT 		=	0xC32;
	public static final int PL_CHAR_JUMP 					=	0xC33;
	public static final int PL_CHAR_APPLY_IMPULSE 			=	0xC34;
	public static final int PL_CHAR_MAX_PENETRATION_DEPTH 	=	0xC35;
	public static final int PL_CHAR_MAX_SLOPE 				=	0xC36;
	public static final int PL_CHAR_CAN_JUMP 				=	0xC37;
	public static final int PL_CHAR_ON_GROUND 				=	0xC38;
	
	/* core Functions */
	public static native boolean plCreateContext();
	public static native void plDestroyContext();
	public static native int plGetError();
	public static native String plGetString(int param);
	public static native int plGetInteger(int param);
	public static native void plCreate(int target);
	public static native void plStepSimulation(float timeStep,int maxSubSteps,float fixedTimeStep);

	/* World Functions */
	public static native void plDynamicWorldi(int param,int value);
	public static native void plDynamicWorld3f(int param,float v1,float v2,float v3);
	public static native void plDynamicWorldfv(int param,float[] values,int lenght);

	public static native int plGetDynamicWorldi(int param);
	public static native void plGetDynamicWorld3f(int param,float[] values);
	public static native void plGetDynamicWorldiv(int param,int[] values,int lenght);
	public static native void plGetDynamicWorldfv(int param,float[] values,int lenght);
	public static native void plSetContactCallBack(plContactCallBack callback);

	/* Body Functions*/
	public static native int plGenBody();
	public static native void plDeleteBody(int body);
	public static native void plBindBody(int body);
	public static native void plRigidBodyi(int param,int value);
	public static native void plRigidBodyf(int param,float value);
	public static native void plRigidBody3f(int param,float v1,float v2,float v3);
	public static native void plRigidBodyfv(int param,float[] values,int lenght);
	
	public static native int plGetRigidBodyi(int body,int param);
	public static native float plGetRigidBodyf(int body,int param);
	public static native void plGetRigidBody3f(int body,int param,float[] values);
	public static native void plGetRigidBodyfv(int body,int param,float[] values,int lenght);

	/* Shape Functions */
	public static native int plGenShape();
	public static native void plBindShape(int shape);
	public static native void plShapei(int param,int value);
	public static native void plShapef(int param,float value);
	public static native void plShape3f(int param,float v1,float v2,float v3);
	public static native void plShapefv(int param,float[] values,int lenght);
	public static native void plBufferData(int type,int size,Buffer data);

	public static native int plGetShapei(int shape,int param);
	public static native void plGetShapefv(int shape,int param,float[] values,int lenght);

	/* Constraint Capability */
	public static native int plGenConstraint(int type);
	public static native void plBindConstraint(int ctr);
	public static native void plDeleteConstraint(int ctr);
	public static native void plConstrainti(int param,int value);
	public static native void plConstraintf(int param,float value);
	public static native void plConstraint3f(int param,float x,float y,float z);
	public static native void plConstraintfv(int param,float[] values,int lenght);

	/* vehicle Ray Casting Extension */
	public static native int plGenVehicle();
	public static native void plBindVehicle(int indx);
	public static native void plDeleteVehicle(int indx);
	public static native void plWheelf(int wheel,int param, float value);
	public static native void plVehiclei(int param,int value);
	public static native void plVehiclef(int param,float value);
	public static native void plVehicle3f(int param,float x,float y,float z);
	public static native void plVehiclefv(int param,float[] values,int lenght);
	public static native void plGetWheelfv(int wheel,int param,float[] values,int lenght);

	/* character extension */
	public static native int plGenCharacter();
	public static native void plBindCharacter(int indx);
	public static native void plDeleteCharacter(int indx);
	public static native void plCharacterf(int param,float val);
	public static native void plCharacter3f(int param,float x,float y,float z);
	public static native void plCharacterfv(int param,float[] values);
	
	public static native float plGetCharacterf(int indx,int param);
	public static native boolean plGetCharacterb(int indx,int param);
	public static native void plGetCharacterfv(int indx,int param,float[] values,int lenght);
	
	static{
		System.loadLibrary("openpl");
	}
}
