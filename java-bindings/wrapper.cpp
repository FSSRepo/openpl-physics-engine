#include <string.h>
#include <jni.h>
#include "PL/pl10.h"

extern "C"
{

	jobject cbo;
	JNIEnv *cbh;
	jclass cbc;

	bool contact_CB(int uptr0, bool m0, int uptr1, bool m1)
	{
		jint ptr0 = (jint)uptr0;
		jint ptr1 = (jint)uptr1;
		jmethodID cbm = cbh->GetMethodID(cbc, "onContact", "(IZIZ)V");
		cbh->CallVoidMethod(cbo, cbm, ptr0, m0, ptr1, m1);
		return true;
	}

	JNIEXPORT jboolean JNICALL Java_com_openpl_PL10_plCreateContext(JNIEnv *env, jobject thiz)
	{
		return (jboolean)plCreateContext();
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plDestroyContext(JNIEnv *env, jobject thiz)
	{
		plDestroyContext();
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plSetContactCallBack(JNIEnv *env, jobject thiz, jobject callback)
	{
		jclass icbc = env->FindClass("com/openpl/plContactCallBack");
		cbc = (jclass)env->NewGlobalRef(icbc);
		cbo = env->NewGlobalRef(callback);
		cbh = env;
		plContactCallBack(contact_CB);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plDynamicWorldi(JNIEnv *env, jobject thiz, jint param, jint value)
	{
		plDynamicWorldi(param, value);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plDynamicWorld3f(JNIEnv *env, jobject thiz, jint param, jfloat v1, jfloat v2, jfloat v3)
	{
		plDynamicWorld3f(param, v1, v2, v3);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plDynamicWorldfv(JNIEnv *env, jobject thiz, jint param, jfloatArray val, jint lenght)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plDynamicWorldfv(param, cval, lenght);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT jint JNICALL Java_com_openpl_PL10_plGetDynamicWorldi(JNIEnv *env, jobject thiz, jint param)
	{
		int value;
		plGetDynamicWorldi(param, &value);
		return (jint)value;
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plGetDynamicWorld3f(JNIEnv *env, jobject thiz, jint param, jfloatArray val)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plGetDynamicWorld3f(param, &cval[0], &cval[1], &cval[2]);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plGetDynamicWorldiv(JNIEnv *env, jobject thiz, jint param, jintArray val, jint lenght)
	{
		int *cval = (int *)env->GetPrimitiveArrayCritical(val, 0);
		plGetDynamicWorldiv(param, cval, lenght);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plGetDynamicWorldfv(JNIEnv *env, jobject thiz, jint param, jfloatArray val, jint lenght)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plGetDynamicWorldfv(param, cval, lenght);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT jint JNICALL Java_com_openpl_PL10_plGetError(JNIEnv *env, jobject thiz)
	{
		return (jint)plGetError();
	}

	JNIEXPORT jstring JNICALL Java_com_openpl_PL10_plGetString(JNIEnv *env, jobject thiz, jint param)
	{
		return env->NewStringUTF(plGetString(param));
	}

	JNIEXPORT jint JNICALL Java_com_openpl_PL10_plGetInteger(JNIEnv *env, jobject thiz, jint param)
	{
		return plGetInteger(param);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plCreate(JNIEnv *env, jobject thiz, jint target)
	{
		plCreate(target);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plBindShape(JNIEnv *env, jobject thiz, jint shape)
	{
		plBindShape(shape);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plBindBody(JNIEnv *env, jobject thiz, jint body)
	{
		plBindBody(body);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plStepSimulation(JNIEnv *env, jobject thiz, jfloat timeStep, jint maxSteps, jfloat fixedTimeStep)
	{
		plStepSimulation(timeStep, maxSteps, fixedTimeStep);
	}

	JNIEXPORT jint JNICALL Java_com_openpl_PL10_plGenShape(JNIEnv *env, jobject thiz)
	{
		return (jint)plGenShape();
	}

	JNIEXPORT jint JNICALL Java_com_openpl_PL10_plGenBody(JNIEnv *env, jobject thiz)
	{
		return (jint)plGenBody();
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plDeleteBody(JNIEnv *env, jobject thiz, jint idx)
	{
		plDeleteBody(idx);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plShapei(JNIEnv *env, jobject thiz, jint param, jint value)
	{
		plShapei(param, value);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plShapef(JNIEnv *env, jobject thiz, jint param, jfloat value)
	{
		plShapef(param, value);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plShape3f(JNIEnv *env, jobject thiz, jint param, jfloat v1, jfloat v2, jfloat v3)
	{
		plShape3f(param, v1, v2, v3);
	}
	JNIEXPORT void JNICALL Java_com_openpl_PL10_plShapefv(JNIEnv *env, jobject thiz, jint param, jfloatArray val, jint lenght)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plShapefv(param, cval, lenght);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT jint JNICALL Java_com_openpl_PL10_plGetShapei(JNIEnv *env, jobject thiz, jint idx, jint param)
	{
		int value;
		plGetShapei(idx, param, &value);
		return (jint)value;
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plGetShapefv(JNIEnv *env, jobject thiz, jint idx, jint param, jfloatArray val, jint lenght)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plGetShapefv(idx, param, cval, lenght);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plBufferData(JNIEnv *env, jobject thiz, jint type, jint size, jobject data)
	{
		plBufferData(type, size, env->GetDirectBufferAddress(data));
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plRigidBodyi(JNIEnv *env, jobject thiz, jint param, jint value)
	{
		plRigidBodyi(param, value);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plRigidBodyf(JNIEnv *env, jobject thiz, jint param, jfloat value)
	{
		plRigidBodyf(param, value);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plRigidBody3f(JNIEnv *env, jobject thiz, jint param, jfloat v1, jfloat v2, jfloat v3)
	{
		plRigidBody3f(param, v1, v2, v3);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plRigidBodyfv(JNIEnv *env, jobject thiz, jint param, jfloatArray val, jint lenght)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plRigidBodyfv(param, cval, lenght);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT jint JNICALL Java_com_openpl_PL10_plGetRigidBodyi(JNIEnv *env, jobject thiz, jint idx, jint param)
	{
		int value;
		plGetRigidBodyi(idx, param, &value);
		return (jint)value;
	}

	JNIEXPORT jfloat JNICALL Java_com_openpl_PL10_plGetRigidBodyf(JNIEnv *env, jobject thiz, jint idx, jint param)
	{
		jfloat value;
		plGetRigidBodyf(idx, param, &value);
		return value;
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plGetRigidBody3f(JNIEnv *env, jobject thiz, jint idx, jint param, jfloatArray val)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plGetRigidBody3f(idx, param, &cval[0], &cval[1], &cval[2]);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plGetRigidBodyfv(JNIEnv *env, jobject thiz, jint idx, jint param, jfloatArray val, jint lenght)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plGetRigidBodyfv(idx, param, cval, lenght);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT jint JNICALL Java_com_openpl_PL10_plGenConstraint(JNIEnv *env, jobject thiz, jint type)
	{
		return (jint)plGenConstraint(type);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plDeleteConstraint(JNIEnv *env, jobject thiz, jint idx)
	{
		plDeleteConstraint(idx);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plBindConstraint(JNIEnv *env, jobject thiz, jint body)
	{
		plBindConstraint(body);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plConstrainti(JNIEnv *env, jobject thiz, jint param, jint value)
	{
		plConstrainti(param, value);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plConstraintf(JNIEnv *env, jobject thiz, jint param, jfloat value)
	{
		plConstraintf(param, value);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plConstraint3f(JNIEnv *env, jobject thiz, jint param, jfloat v1, jfloat v2, jfloat v3)
	{
		plConstraint3f(param, v1, v2, v3);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plConstraintfv(JNIEnv *env, jobject thiz, jint param, jfloatArray val, jint lenght)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plConstraintfv(param, cval, lenght);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT jint JNICALL Java_com_openpl_PL10_plGenVehicle(JNIEnv *env, jobject thiz)
	{
		return (jint)plGenVehicle();
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plBindVehicle(JNIEnv *env, jobject thiz, jint veh)
	{
		plBindVehicle(veh);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plDeleteVehicle(JNIEnv *env, jobject thiz, jint idx)
	{
		plDeleteVehicle(idx);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plVehiclei(JNIEnv *env, jobject thiz, jint param, jint value)
	{
		plVehiclei(param, value);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plVehiclef(JNIEnv *env, jobject thiz, jint param, jfloat value)
	{
		plVehiclef(param, value);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plVehicle3f(JNIEnv *env, jobject thiz, jint param, jfloat v1, jfloat v2, jfloat v3)
	{
		plVehicle3f(param, v1, v2, v3);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plVehiclefv(JNIEnv *env, jobject thiz, jint param, jfloatArray val, jint lenght)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plVehiclefv(param, cval, lenght);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plWheelf(JNIEnv *env, jobject thiz, jint wheel, jint param, jfloat value)
	{
		plWheelf(wheel, param, value);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plGetWheelfv(JNIEnv *env, jobject thiz, jint wheel, jint param, jfloatArray val, jint lenght)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plGetWheelfv(wheel, param, cval, lenght);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT jint JNICALL Java_com_openpl_PL10_plGenCharacter(JNIEnv *env, jobject thiz)
	{
		return (jint)plGenCharacter();
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plBindCharacter(JNIEnv *env, jobject thiz, jint chr)
	{
		plBindCharacter(chr);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plDeleteCharacter(JNIEnv *env, jobject thiz, jint idx)
	{
		plDeleteCharacter(idx);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plCharacterf(JNIEnv *env, jobject thiz, jint param, jfloat value)
	{
		plCharacterf(param, value);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plCharacter3f(JNIEnv *env, jobject thiz, jint param, jfloat v1, jfloat v2, jfloat v3)
	{
		plCharacter3f(param, v1, v2, v3);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plCharacterfv(JNIEnv *env, jobject thiz, jint param, jfloatArray val)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plCharacterfv(param, cval);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}

	JNIEXPORT jfloat JNICALL Java_com_openpl_PL10_plGetCharacterf(JNIEnv *env, jobject thiz, jint chr, jint param)
	{
		jfloat value;
		plGetCharacterf(chr, param, &value);
		return value;
	}

	JNIEXPORT jboolean JNICALL Java_com_openpl_PL10_plGetCharacterb(JNIEnv *env, jobject thiz, jint chr, jint param)
	{
		return (jboolean)plGetCharacterb(chr, param);
	}

	JNIEXPORT void JNICALL Java_com_openpl_PL10_plGetCharacterfv(JNIEnv *env, jobject thiz, jint chr, jint param, jfloatArray val, jint lenght)
	{
		float *cval = (float *)env->GetPrimitiveArrayCritical(val, 0);
		plGetCharacterfv(chr, param, cval, lenght);
		env->ReleasePrimitiveArrayCritical(val, cval, 0);
	}
}
