/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 1
#define ARRAY_SIZE_Y 1
#define ARRAY_SIZE_Z 1

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X 5
#define START_POS_Y 3
#define START_POS_Z 5

#include <GL/glew.h>
#include "SPHDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
//#include <RTPS.h>
//#include <RTPSettings.h>
#include "../GimpactTestDemo/BunnyMesh.h"

static GLuint gIndices[BUNNY_NUM_TRIANGLES*3];


#include <stdio.h> //printf debugging
#include "stb_image_write.h"


void SPHDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
        
	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme();
        
        //voxelizeMesh(m_dynamicsWorld->getCollisionObjectArray().at(0)->getCollisionShape(),256);
	glFlush();


	swapBuffers();

}



void SPHDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();
        
        //voxelizeMesh(m_dynamicsWorld->getCollisionObjectArray().at(0)->getCollisionShape(),256);
	glFlush();
	swapBuffers();
}

void    SPHDemo::renderme()
{
        DemoApplication::renderme();
        if (m_rtps)
        {
            m_rtps->update();
            m_rtps->render();
        }
}



void	SPHDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(SCALING*10.));

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
        //btParallelConstraintSolver* sol = new btParallelConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
            for(int i = 0; i<BUNNY_NUM_TRIANGLES;i++)
            {
                gIndices[(i*3)]=gIndicesBunny[i][0];
                gIndices[(i*3)+1]=gIndicesBunny[i][1];
                gIndices[(i*3)+2]=gIndicesBunny[i][2];
            }
                btIndexedMesh* mesh = new btIndexedMesh;
                mesh->m_numTriangles = BUNNY_NUM_TRIANGLES;
                mesh->m_numVertices = BUNNY_NUM_VERTICES;
                mesh->m_triangleIndexBase = (unsigned char*)gIndices;
                mesh->m_triangleIndexStride = 3*sizeof(int);
                mesh->m_vertexBase = (unsigned char*)gVerticesBunny;
                mesh->m_vertexStride = 3*sizeof(btScalar);
                btTriangleIndexVertexArray* tmesh = new btTriangleIndexVertexArray();
                tmesh->addIndexedMesh(*mesh);
		//btCollisionShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
        btCollisionShape* colShape = new btBvhTriangleMeshShape(tmesh,false);
		m_collisionShapes.push_back(colShape);
		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(10.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		//if (isDynamic)
		//	colShape->calculateLocalInertia(mass,localInertia);

		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(SCALING*btVector3(
										btScalar(2.0*i + start_x),
										btScalar(5+2.0*k + start_y),
										btScalar(2.0*j + start_z)));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					

					m_dynamicsWorld->addRigidBody(body);
				}
			}
		}
	}

}

//Use this function to initialize RTPS. Because this is called after glutInit()
//Which is important for cl/gl interop. This initialization will change when
//I integrate it better into Bullet. For now I just need something that works.
//-ASY 09/19/2011
void    SPHDemo::myinit()
{
    DemoApplication::myinit();

    if(!m_rtps)
    {
        rtps::Domain* grid = new rtps::Domain(rtps::float4(0,0,0,0), rtps::float4(10, 10, 10, 0));
        rtps::RTPSettings* settings = new rtps::RTPSettings(rtps::RTPSettings::SPH, m_numParticles, m_timestep, grid);
    

        //should be argv[0]
        #ifdef WIN32
            settings->SetSetting("rtps_path", ".");
        #else
            //FIXME: This should not be hard coded!!! -ASY 09/19/2011
            settings->SetSetting("rtps_path", "/home/andrew/School/Research/Bullet/BulletPhysics_SPH/Demos/SPHDemo/EnjaParticles/build/bin/");
        #endif

        //settings->setRenderType(rtps::RTPSettings::SCREEN_SPACE_RENDER);
        settings->setRenderType(rtps::RTPSettings::RENDER);
        //settings.setRenderType(RTPSettings::SPRITE_RENDER);
        settings->setRadiusScale(0.4);
        settings->setBlurScale(1.0);
        settings->setUseGLSL(1);

        settings->SetSetting("sub_intervals", 1);
        settings->SetSetting("render_use_alpha", true);
        //settings->SetSetting("render_use_alpha", false);
        settings->SetSetting("render_alpha_function", "add");
        settings->SetSetting("lt_increment", -.00);
        settings->SetSetting("lt_cl", "lifetime.cl");

        m_rtps = new rtps::RTPS(settings);
        //m_rtps = new rtps::RTPS();


        m_rtps->settings->SetSetting("Gravity",-1.0f); // -9.8 m/sec^2
        //m_rtps->settings->SetSetting("Gravity", -9.8f); // -9.8 m/sec^2
        m_rtps->settings->SetSetting("Gas Constant", 1.0f);
        m_rtps->settings->SetSetting("Viscosity", .001f);
        m_rtps->settings->SetSetting("Velocity Limit", 600.0f);
        m_rtps->settings->SetSetting("XSPH Factor", .15f);
        m_rtps->settings->SetSetting("Friction Kinetic", 0.0f);
        m_rtps->settings->SetSetting("Friction Static", 0.0f);
        m_rtps->settings->SetSetting("Boundary Stiffness", 20000.0f);
        m_rtps->settings->SetSetting("Boundary Dampening", 256.0f);
        //int nn = 2048;
        rtps::float4 min = rtps::float4(.1, .1, .1, 1.0f);
        rtps::float4 max = rtps::float4(3.9, 3.9, 3.9, 1.0f);
        rtps::float4 color = rtps::float4(0.f, .5f,.75f, .05f);
        m_rtps->system->addBox(m_numParticles/4, min, max, false,color);
    }
    if(!m_voxelizedMesh)
    {
        voxelizeMesh(m_dynamicsWorld->getCollisionObjectArray().at(1),64);
        m_voxelizedMesh=true;    
    }
}

void	SPHDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}
	

void	SPHDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;
}

void write3DTextureToDisc(GLuint tex,int voxelResolution, const char* filename)
{
    printf("writing %s texture to disc.\n",filename);
    glBindTexture(GL_TEXTURE_3D_EXT,tex);
    GLubyte* image = new GLubyte[voxelResolution*voxelResolution*voxelResolution*4];
    //GLubyte* image = new GLubyte[voxelResolution*voxelResolution*voxelResolution*3];
    glGetTexImage(GL_TEXTURE_3D_EXT,0,GL_RGBA,GL_UNSIGNED_BYTE,image);
    //glGetTexImage(GL_TEXTURE_3D_EXT,0,GL_RGB,GL_UNSIGNED_BYTE,image);
    GLubyte* tmp2Dimg = new GLubyte[voxelResolution*voxelResolution*4];
    //GLubyte* tmp2Dimg = new GLubyte[voxelResolution*voxelResolution*3];
    for(int i = 0; i<voxelResolution; i++)
    {
        char fname[128];
        sprintf(fname,"%s-%03d.png",filename,i);
        memcpy(tmp2Dimg,image+(i*(voxelResolution*voxelResolution*4)),sizeof(GLubyte)*voxelResolution*voxelResolution*4);
        //memcpy(tmp2Dimg,image+(i*(voxelResolution*voxelResolution*3)),sizeof(GLubyte)*voxelResolution*voxelResolution*3);
        if (!stbi_write_png(fname,voxelResolution,voxelResolution,4,(void*)tmp2Dimg,0))
        //if (!stbi_write_png(fname,voxelResolution,voxelResolution,3,(void*)tmp2Dimg,0))
        {
            printf("failed to write image %s",filename);
        }
    }
    glBindTexture(GL_TEXTURE_3D_EXT,0);
    delete[] image;

}

void SPHDemo::voxelizeMesh(btCollisionObject* colObj, int voxelResolution)
{
        btClock clk;
        btVector3 min,max;
        btCollisionShape* shape = colObj->getCollisionShape();
        shape->getAabb(btTransform::getIdentity(),min,max);
        printf("min = (%f,%f,%f)\n",min.x(),min.y(),min.z());
        printf("max = (%f,%f,%f)\n",max.x(),max.y(),max.z());
        btVector3 dim = (max-min);
        //printf("dim = (%f,%f,%f)\n",dim.x(),dim.y(),dim.z());
        
        printf("3d texture supported? %d\n",glewIsSupported("GL_EXT_texture3D"));
        glEnable(GL_TEXTURE_3D_EXT);
        //glEnable(GL_DRAW_BUFFER);
        //glEnable(GL_FRAMEBUFFER);
        GLuint tex=0;
        glGenTextures(1, &tex);
        printf("tex = %d\n",tex);
        glBindTexture(GL_TEXTURE_3D_EXT, tex);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glTexParameteri(GL_TEXTURE_3D_EXT, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_3D_EXT, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        GLint mode = GL_CLAMP_TO_BORDER;
        glTexParameteri(GL_TEXTURE_3D_EXT, GL_TEXTURE_WRAP_S, mode);
        glTexParameteri(GL_TEXTURE_3D_EXT, GL_TEXTURE_WRAP_T, mode);
        glTexParameteri(GL_TEXTURE_3D_EXT, GL_TEXTURE_WRAP_R, mode);
        //img is for debugging
        //unsigned char img[voxelResolution*voxelResolution*voxelResolution*4];
        //memset(img,0,sizeof(unsigned char)*voxelResolution*voxelResolution*voxelResolution*4);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage3DEXT(GL_TEXTURE_3D_EXT, 0, GL_RGBA, voxelResolution, voxelResolution, voxelResolution, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);//img);
        GLuint depth=0;
        glGenTextures(1, &depth);
        glBindTexture(GL_TEXTURE_2D, depth);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT32,voxelResolution,voxelResolution,0,GL_DEPTH_COMPONENT,GL_FLOAT,NULL);
        
        glViewport(0,0,voxelResolution,voxelResolution);

        GLuint fboId = 0;
        glGenFramebuffersEXT(1, &fboId);
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,fboId);
        //glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT,fboId);
        glFramebufferTextureLayer( GL_DRAW_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT , tex, 0,0);
        glFramebufferTexture2DEXT(GL_DRAW_FRAMEBUFFER_EXT,GL_DEPTH_ATTACHMENT_EXT,GL_TEXTURE_2D,depth,0);
        float col[4];
        glGetFloatv(GL_COLOR_CLEAR_VALUE,col);
        glClearColor(0.0f,0.0f,0.0f,0.0f);

        btScalar maxDim = max.x();
        if(maxDim<max.y())
            maxDim=max.y();
        if(maxDim<max.z())
            maxDim=max.z();
        

        double delz = (2.0*maxDim)/voxelResolution;
        int i = 0;
        //FIXME: Code should check and preserve the current state so that
        //It correctly restores previous state.
        glEnable(GL_COLOR_LOGIC_OP);
        glEnablei(GL_BLEND,0);//GL_DRAW_BUFFER0);
        //glBlendFunc(GL_SRC_COLOR,GL_DST_COLOR);
        glLogicOp(GL_NOR);
        glBlendFunc(GL_ONE,GL_ONE);
        glDisable(GL_ALPHA_TEST);
        glDisable(GL_POINT_SMOOTH);
        glDisable(GL_LINE_SMOOTH);
        
        /*
        **/
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glDisable(GL_TEXTURE_2D);
        glEnableClientState(GL_VERTEX_ARRAY);
        
        GLenum status = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER_EXT);
        if(status!=GL_FRAMEBUFFER_COMPLETE)
        {
            switch(status)
            {
                case GL_FRAMEBUFFER_UNDEFINED:
                    printf("Here LINE %d\n",__LINE__);
                    break;
                case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
                    printf("Here LINE %d\n",__LINE__);
                    break;
                case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
                    printf("Here LINE %d\n",__LINE__);
                    break;
                case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
                    printf("Here LINE %d\n",__LINE__);
                    break;
                case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
                    printf("Here LINE %d\n",__LINE__);
                    break;
                case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:       
                    printf("Here LINE %d\n",__LINE__);
                    break;
                default:
                    printf("WTF??\n");
                    break;
            }
        }
        else
        {
            printf("FRAMEBUFFER is complete.\n");
        }
        unsigned long int start = clk.getTimeMilliseconds();
        glMatrixMode(GL_PROJECTION);
        // save previous matrix which contains the 
        //settings for the perspective projection
        glPushMatrix();
        glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
        glReadBuffer(GL_COLOR_ATTACHMENT1_EXT);
        //m_shapeDrawer->enableTexture(false);
        for(int i = 0; i<voxelResolution; i++)
        {
            glFramebufferTextureLayer( GL_DRAW_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT , tex, 0, i );
            glClear(GL_COLOR_BUFFER_BIT);
            glFramebufferTextureLayer(GL_READ_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, tex, 0, i?i-1:0);
            glBlitFramebuffer(0,0,voxelResolution,voxelResolution,
                                0,0,voxelResolution,voxelResolution,
                                GL_COLOR_BUFFER_BIT,GL_NEAREST);
            glMatrixMode(GL_PROJECTION);
            // switch to projection mode
            // reset matrix
            glLoadIdentity();
            // set a 2D orthographic projection
            //glOrtho(0, maxDim, 0, maxDim,delz*i*maxDim,delz*(i+1)*maxDim);
            glOrtho(-maxDim, maxDim, -maxDim, maxDim ,-maxDim+delz*i ,-maxDim+delz*(i+1));
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();

            //glTranslatef(0.f,0.f,-maxDim);
            //btScalar m[16];
            //btTransform::getIdentity().getOpenGLMatrix(m);
            //btVector3 a(btScalar(1.0),btScalar(1.0),btScalar(1.0));
            //m_shapeDrawer->drawOpenGL(m,shape,a,getDebugMode(),min,max);
            glVertexPointer( 3,   //3 components per vertex (x,y,z)
                 GL_FLOAT,
                 sizeof(btScalar)*3,
                 gVerticesBunny);
            //glBegin();
            glColor4f(1.0f,1.0f,1.0f,1.0f);
            //glDrawElements( GL_TRIANGLE_STRIP, //mode
            glDrawElements( GL_TRIANGLES, //mode
                                        BUNNY_NUM_INDICES,  //count, ie. how many indices
                                        //GL_INT, //type of the index array
                                        GL_UNSIGNED_INT, //type of the index array
                                        //gIndicesBunny);
                                        gIndices);
            //glEnd();
            glPopMatrix();
            glFlush();
            //glMatrixMode(GL_PROJECTION);
            //glPopMatrix();
        }
        write3DTextureToDisc(tex,voxelResolution,"test");
        //m_shapeDrawer->enableTexture(true);
        glViewport(0,0,m_glutScreenWidth,m_glutScreenHeight);
        glDisableClientState(GL_VERTEX_ARRAY);
        glEnable(GL_ALPHA_TEST);
        glEnable(GL_TEXTURE_2D);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glLogicOp(GL_COPY);
        glEnable(GL_CULL_FACE);
        glDisable(GL_COLOR_LOGIC_OP);
        /*
        */
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        unsigned long int end = clk.getTimeMilliseconds();
//printf("Here at %s:%d\n",__FILE__,__LINE__);
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);
        //glDrawBuffer(0);
        //glReadBuffer(0);
        glClearColor(col[0],col[1],col[2],col[3]);
        glMatrixMode(GL_MODELVIEW);
        rtps::float4 ext = rtps::float4(dim.x(),dim.y(),dim.z(),0.0);
        rtps::float4 minimum = rtps::float4(min.x(),min.y(),min.z(),0.0);
        btRigidBody*		body=btRigidBody::upcast(colObj);
        btScalar m[16];
        if(body&&body->getMotionState())
        {
            btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
            btTransform offset;
            myMotionState->getWorldTransform(offset);
            offset.getOpenGLMatrix(m);
        }
        rtps::float16 m2;
        //NOTE: Should we need to transpose the matrix?
        for(int i =0;i<4;i++){
            for(int j = 0; j<4; j++){
                m2.m[j+i*4]= m[i+j*4];
            }
        }
        //memcpy(m2.m,m,sizeof(rtps::float16));
        m_rtps->system->addRigidBody(tex,ext,minimum,m2,voxelResolution);
        glDeleteTextures(1,&tex);
        glDeleteFramebuffersEXT(1,&fboId);
        glDisable(GL_TEXTURE_3D_EXT);
        printf("time to voxelize = %ld\n",end-start);
}