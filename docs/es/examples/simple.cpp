#include <openpl/bullet.h>
#include <GL/glew.h>
#include
#include <iostream>
using namespace std;

/*
	Fast Smart System 
	cross-platform open source project
*/

int main(int* args0){
	if(!plCreateContext()){
		cout << "Can't the context" << endl;
	}
	
	int shape = plGenShape();
	plShapef(PL_RADIUS,3.0f);
	plCreate(PL_SPHERE_SHAPE);
	plBindShape(shape);
	
	plBindShape(0);
	plDestroyContext();
}
