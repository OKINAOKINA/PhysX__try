//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2021 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet illustrates simple use of joints in physx
//
// It creates a chain of objects joined by limited spherical joints, a chain
// joined by fixed joints which is breakable, and a chain of damped D6 joints
// ****************************************************************************

#include <ctype.h>
#include <cmath>

#include "PxPhysicsAPI.h"

#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"


using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene		= NULL;

PxMaterial*				gMaterial	= NULL;

PxPvd*                  gPvd        = NULL;

PxRigidDynamic* bullet = NULL;

PxReal chainZ = 10.0f;

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
	if (bullet != NULL)
	{
		gScene->removeActor(*bullet);
	}
	PxTransform s = PxTransform(t.p, t.q * PxQuat(0,sin(PxPi / 4),0,  cos(PxPi / 4)));
	bullet = PxCreateDynamic(*gPhysics, s, geometry, *gMaterial, 2e20);
	bullet->setAngularDamping(0.005f);
	bullet->setLinearVelocity(velocity);
	bullet->setName("bullet");
	gScene->addActor(*bullet);
	return bullet;
}

// spherical joint limited to an angle of at most pi/4 radians (45 degrees)
PxJoint* createLimitedSpherical(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
{
	PxSphericalJoint* j = PxSphericalJointCreate(*gPhysics, a0, t0, a1, t1);
	j->setLimitCone(PxJointLimitCone(PxPi/4, PxPi/4, 0.05f));
	j->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
	return j;
}

// revolute joint limited to an angle of at most pi/4 radians (45 degrees)

// fixed, breakable joint
PxJoint* createBreakableFixed(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1, PxReal x)
{
	PxFixedJoint* j = PxFixedJointCreate(*gPhysics, a0, t0, a1, t1);
	j->setBreakForce(2e3 * x, 6e4 * x);
	j->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
	j->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);
	return j;
}

// D6 joint with a spring maintaining its position
PxJoint* createDampedD6(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1, PxReal x)
{
	PxD6Joint* j = PxD6JointCreate(*gPhysics, a0, t0, a1, t1);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
	j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
	j->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(10000, 10000, FLT_MAX, true));
	j->setBreakForce(3e3 * x , 3e4 * x);

	j->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
	j->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);
	return j;
}

typedef PxJoint* (*JointCreateFunction)(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1);

// create a chain rooted at the origin and extending along the x-axis, all transformed by the argument t.

//void createChain(const PxTransform& t, PxU32 length, const PxGeometry& g, PxReal separation, JointCreateFunction createJoint)
//{
//	PxVec3 offset(separation/2, 0, 0);
//	PxTransform localTm(offset);
//	PxRigidDynamic* prev = NULL;
//
//	for(PxU32 i=0;i<length;i++)
//	{
//		PxRigidDynamic* current = PxCreateDynamic(*gPhysics, t*localTm, g, *gMaterial, 1.0f);
//		(*createJoint)(prev, prev ? PxTransform(offset) : t, current, PxTransform(-offset));
//		gScene->addActor(*current);
//		prev = current;
//		localTm.p.x += separation;
//	}
//}

void createChain(const PxVec3 t, const PxGeometry& g, PxReal sep, PxReal xx)
{
	//const int x = 5;
	//const int y = 5;
	//const int z = 5;

	const int x = 7;
	const int y = 21;
	const int z = 7;

	PxRigidDynamic* chain[x][y][z];

	for (PxU32 i = 0; i < x; i++)
	{
		for (PxU32 j = 0; j < y; j++)
		{
			for (PxU32 k = 0; k < z; k++)
			{
				chain[i][j][k] = PxCreateDynamic(*gPhysics, PxTransform(t + PxVec3(sep * i, sep * j, sep * k)), g, *gMaterial, 1);
				gScene->addActor(*chain[i][j][k]);
			}
		}
	}

	for (PxU32 i = 0; i < x; i++)
	{
		for (PxU32 j = 0; j < y; j++)
		{
			for (PxU32 k = 0; k < z; k++)
			{

				if (i < x - 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(sep / 2, 0, 0)), chain[i + 1][j][k], PxTransform(PxVec3(-sep / 2, 0, 0)), std::pow(0.95, i + j + k));
				if (j < y - 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(0, sep / 2, 0)), chain[i][j + 1][k], PxTransform(PxVec3(0, -sep / 2, 0)), std::pow(0.95, i + j + k));
				if (k < z - 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(0, 0, sep / 2)), chain[i][j][k + 1], PxTransform(PxVec3(0, 0, -sep / 2)), std::pow(0.95, i + j + k));



				if (i < x - 1 && j < y - 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(sep / 2, sep / 2, 0)), chain[i + 1][j + 1][k], PxTransform(PxVec3(-sep / 2, -sep / 2, 0)), std::pow(0.95, i + j + k));
				if (j < y - 1 && k < z - 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(0, sep / 2, sep / 2)), chain[i][j + 1][k + 1], PxTransform(PxVec3(0, -sep / 2, -sep / 2)), std::pow(0.95, i + j + k));
				if (k < z - 1 && i < x - 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(sep / 2, 0, sep / 2)), chain[i + 1][j][k + 1], PxTransform(PxVec3(-sep / 2, 0, -sep / 2)), std::pow(0.95, i + j + k));


				if (i > 1 && j < y - 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(-sep / 2, sep / 2, 0)), chain[i - 1][j + 1][k], PxTransform(PxVec3(sep / 2, -sep / 2, 0)), std::pow(0.95, i + j + k));
				if (j > 1 && k < z - 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(0, -sep / 2, sep / 2)), chain[i][j - 1][k + 1], PxTransform(PxVec3(0, sep / 2, -sep / 2)), std::pow(0.95, i + j + k));
				if (k > 1 && i < x - 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(sep / 2, 0, -sep / 2)), chain[i + 1][j][k - 1], PxTransform(PxVec3(-sep / 2, 0, sep / 2)), std::pow(0.95, i + j + k));


				if (i < x - 1 && j < y - 1 && k < z - 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(sep / 2, sep / 2, sep / 2)), chain[i + 1][j + 1][k + 1], PxTransform(PxVec3(-sep / 2, -sep / 2, -sep / 2)), std::pow(0.95, i + j + k));
				if (i < x - 1 && j > 1 && k < z - 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(sep / 2, -sep / 2, sep / 2)), chain[i + 1][j - 1][k + 1], PxTransform(PxVec3(-sep / 2, sep / 2, -sep / 2)), std::pow(0.95, i + j + k));
				if (i < x - 1 && j < y - 1 && k > 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(sep / 2, sep / 2, -sep / 2)), chain[i + 1][j + 1][k - 1], PxTransform(PxVec3(-sep / 2, -sep / 2, sep / 2)), std::pow(0.95, i + j + k));
				if (i < x - 1 && j > 1 && k > 1)
					createDampedD6(chain[i][j][k], PxTransform(PxVec3(sep / 2, -sep / 2, -sep / 2)), chain[i + 1][j - 1][k - 1], PxTransform(PxVec3(-sep / 2, sep / 2, sep / 2)), std::pow(0.95, i + j + k));
			}
		}
	}
}	

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(0.999, 0.999, 0.001);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

	//createChain(PxTransform(PxVec3(0.0f, 20.0f, 0.0f)), 5, PxBoxGeometry(2.0f, 0.5f, 0.5f), 4.0f, createLimitedSpherical);
	//createChain(PxTransform(PxVec3(0.0f, 20.0f, -10.0f)), 5, PxBoxGeometry(2.0f, 0.5f, 0.5f), 4.0f, createBreakableFixed);
	//createChain(PxTransform(PxVec3(0.0f, 20.0f, -20.0f)), 5, PxBoxGeometry(2.0f, 0.5f, 0.5f), 4.0f, createDampedD6);

	const PxReal l = 0.5;
	const PxReal s = 2;
	const PxReal p = 2.5;

	createChain(PxVec3(0, l, 0), PxBoxGeometry(l, l, l), l * s, 0.1 * std::pow(p, 4));
	//createChain(PxVec3(0, l,  0), PxBoxGeometry(l, l, l), l * s, 0.1 * std::pow(p, 0));
	//createChain(PxVec3(0, l, 10), PxBoxGeometry(l, l, l), l * s, 0.1 * std::pow(p, 1));
	//createChain(PxVec3(0, l, 20), PxBoxGeometry(l, l, l), l * s, 0.1 * std::pow(p, 2));
	//createChain(PxVec3(0, l, 30), PxBoxGeometry(l, l, l), l * s, 0.1 * std::pow(p, 3));
	//createChain(PxVec3(0, l, 40), PxBoxGeometry(l, l, l), l * s, 0.1 * std::pow(p, 4));
	//createChain(PxVec3(0, l, 50), PxBoxGeometry(l, l, l), l * s, 0.1 * std::pow(p, 5));

	{
		PxRigidDynamic* wall = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(100, 100, 0)), PxBoxGeometry(10, 100, 100), *gMaterial, 1);
		wall->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		gScene->addActor(*wall);
	}
	{
		PxRigidDynamic* wall = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(-100, 100, 0)), PxBoxGeometry(10, 100, 100), *gMaterial, 1);
		wall->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		gScene->addActor(*wall);
	}
	{
		PxRigidDynamic* wall = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(0, 100, 100)), PxBoxGeometry(100, 100, 10), *gMaterial, 1);
		wall->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		gScene->addActor(*wall);
	}
	{
		PxRigidDynamic* wall = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(0, 100, -100)), PxBoxGeometry(100, 100, 10), *gMaterial, 1);
		wall->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		gScene->addActor(*wall);
	}
	{
		PxRigidDynamic* wall = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(0, 200, 0)), PxBoxGeometry(100, 10, 100), *gMaterial, 1);
		wall->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		gScene->addActor(*wall);
	}
}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}
	
void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PxCloseExtensions();
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
	
	printf("SnippetJoint done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch (toupper(key))
	{
	case ' ':	createDynamic(camera, PxCapsuleGeometry(0.75, 2.5), camera.rotate(PxVec3(0, 0, -1)) * 100);	break;
	case 'V':	bullet = NULL;  cleanupPhysics(true); initPhysics(true);	break;
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
