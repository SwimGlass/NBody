#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <GL/glut.h>
#include "hsa.h"
#include "hsa_ext_finalize.h"

//#define GLOBAL_SIZE 1024*1024
#define GLOBAL_SIZE 10
//#define LOCAL_SIZE 512
#define LOCAL_SIZE 1

#define check(msg, status) \
   if (status != HSA_STATUS_SUCCESS) { \
	  printf("%s failed.\n", #msg); \
	  exit(1); \
   } else { \
	  printf("%s succeeded.\n", #msg); \
   }

//Data type for kernel argument
typedef struct float4 {
   float x;
   float y;
   float z;
   float w;
}float4;

//Global variable
hsa_status_t err;
int numFrames = 0;
float4 *pos,*vel;
float4 newPos[10],newVel[10];

void init(void)
{
    glClearColor(0.0 ,0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glClear(GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
}

void idle()
{
    glutPostRedisplay();
}

void reShape(int w,int h)
{
    glViewport(0, 0, w, h);

    glViewport(0, 0, w, h);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluPerspective(45.0f, w/h, 1.0f, 1000.0f);
    gluLookAt (0.0, 0.0, -2.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0);
}

void displayfunc()
{

    glClearColor(0.0 ,0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glClear(GL_DEPTH_BUFFER_BIT);

    //glPointSize(1.0);
    glPointSize(10.0);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glEnable(GL_BLEND);
    glDepthMask(GL_FALSE);

    glColor3f(1.0f, 0.5f, 0.5f);

	//NBody *nb = (NBody *)me;
    /*if (nb->isFirstLuanch)
    {
        //Calling kernel for calculatig subsequent positions
        nb->runCLKernels();
        nb->isFirstLuanch = false;
        return;
    }*/


    //int numBodies = nb->numParticles;
    int numBodies = 10;
	//float* pos = nb->getMappedParticlePositions();
    //nb->runCLKernels();
    glBegin(GL_POINTS);
    for(int i = 0; i < numBodies; ++i,pos+=4)
    {
        //divided by 300 just for scaling
        glVertex4f(pos[i].x,pos[i].y,pos[i].z,300.0f);
    }
    glEnd();
    //nb->releaseMappedParticlePositions();

    //Calling kernel for calculating subsequent positions
    glFlush();
    glutSwapBuffers();

    numFrames++;
    // update window title with FPS
    /*if (numFrames >= 100)
    {
        char buf[256];
        sprintf(buf, "N-body simulation - %d Particles, %.02f FPS"
                , nb->numParticles, (float)nb->getFPS());
        glutSetWindowTitle(buf);
        numFrames = 0;
    }*/
}



float random(float randMax, float randMin){
	float result;
	result =(float)rand() / (float)RAND_MAX;
	return ((1.0f - result) * randMin + result *randMax);
}

void nBodyCPUReference(float4 *currentPos, float4 *currentVel, float4 *newPos,float4 *newVel)
{
	int numBodies = 10;
	float epsSqr = 500 , delT = 0.005;
   //Iterate for all samples
    for(int i = 0; i < numBodies; ++i)
    {
        //int myIndex = 4 * i;
		float acc[3] = {0.0f, 0.0f, 0.0f};
        for(int j = 0; j < numBodies; ++j)
        {
            float r[3];
            //int index = 4 * j;
            float distSqr = 0.0f;
			int k = 0;
            while(k <3)
            {
                //r[k] = currentPos[index + k] - currentPos[myIndex + k];
				if(k == 0) r[k++] = currentPos[j].x-currentPos[i].x;
				else if(k == 1) r[k++] = currentPos[j].y-currentPos[i].y;
				else if(k == 2) r[k++] = currentPos[j].z-currentPos[i].z;
                distSqr += r[k] * r[k];
            }

            float invDist = 1.0f / sqrt(distSqr + epsSqr);
            float invDistCube =  invDist * invDist * invDist;
            float s = currentPos[j].w * invDistCube;
            for( k = 0; k < 3; ++k)
            {
                acc[k] += s * r[k];
            }
        }
		int k;
        for(k = 0; k < 3; ++k)
        {
            //newPos[myIndex+k] = currentPos[myIndex+k] + currentVel[myIndex + k]* delT + 0.5f * acc[k] * delT * delT;
			if(k==0)newPos[i].x = currentPos[i].x + currentVel[i].x * delT + 0.5f * acc[k] * delT * delT;
			else if(k==1)newPos[i].y = currentPos[i].y + currentVel[i].y * delT + 0.5f * acc[k] * delT * delT;
			else if(k==2)newPos[i].z = currentPos[i].z + currentVel[i].z * delT + 0.5f * acc[k] * delT * delT;
            //newVel[myIndex+k] = currentVel[myIndex+k] + acc[k] * delT;
			if(k ==0) newVel[i].x = currentVel[i].x + acc[k] * delT;
			if(k ==1) newVel[i].y = currentVel[i].y + acc[k] * delT;
			if(k ==2) newVel[i].z = currentVel[i].z + acc[k] * delT;
        }
        newPos[i].w = currentPos[i].w;
    }
}



int loopcounter=0;

//particle initalize
void nbody_init(float4* pos, float4* vel){
   int i;
   for(i=0;i<10;i++){
	  pos[i].x = random(3,50);
	  pos[i].y = random(3,50);
	  pos[i].z = random(3,50);
	  pos[i].w = 300;
	  vel[i].x = random(1,1000);
	  vel[i].y = random(1,1000);
	  vel[i].z = random(1,1000);
	  vel[i].w = 0;
   }
}

/*
 * Loads a BRIG module from a specified file. This
 * function does not validate the module.
 */
int load_module_from_file(const char* file_name, hsa_ext_module_t* module) {
   int rc = -1;
   FILE *fp = fopen(file_name, "rb");
   assert(fp != NULL);
   rc = fseek(fp, 0, SEEK_END);
   size_t file_size = (size_t) (ftell(fp) * sizeof(char));
   rc = fseek(fp, 0, SEEK_SET);
   char* buf = (char*) malloc(file_size);
   memset(buf,0,file_size);
   size_t read_size = fread(buf,sizeof(char),file_size,fp);
   if(read_size != file_size) {
	  free(buf);
   } else {
	  rc = 0;
	  *module = (hsa_ext_module_t) buf;
   }
   fclose(fp);
   return rc;
}
/*
 * Determines if the given agent is of type HSA_DEVICE_TYPE_GPU
 * and sets the value of data to the agent handle if it is.
 */
static hsa_status_t get_gpu_agent(hsa_agent_t agent, void *data) {
   hsa_status_t status;
   hsa_device_type_t device_type;
   status = hsa_agent_get_info(agent, HSA_AGENT_INFO_DEVICE, &device_type);
   if (HSA_STATUS_SUCCESS == status && HSA_DEVICE_TYPE_GPU == device_type) {
	  hsa_agent_t* ret = (hsa_agent_t*)data;
	  *ret = agent;
	  return HSA_STATUS_INFO_BREAK;
   }
   return HSA_STATUS_SUCCESS;
}

/*
 * Determines if a memory region can be used for kernarg
 * allocations.
 */
static hsa_status_t get_kernarg_memory_region(hsa_region_t region, void* data) {
   hsa_region_segment_t segment;
   hsa_region_get_info(region, HSA_REGION_INFO_SEGMENT, &segment);
   if (HSA_REGION_SEGMENT_GLOBAL != segment) {
	  return HSA_STATUS_SUCCESS;
   }

   hsa_region_global_flag_t flags;
   hsa_region_get_info(region, HSA_REGION_INFO_GLOBAL_FLAGS, &flags);
   if (flags & HSA_REGION_GLOBAL_FLAG_KERNARG) {
	  hsa_region_t* ret = (hsa_region_t*) data;
	  *ret = region;
	  return HSA_STATUS_INFO_BREAK;
   }

   return HSA_STATUS_SUCCESS;
}


int main(int argc, char **argv) {

   err = hsa_init();
   check(Initializing the hsa runtime, err);

   /* 
	* Iterate over the agents and pick the gpu agent using 
	* the get_gpu_agent callback.
	*/
   hsa_agent_t agent;
   err = hsa_iterate_agents(get_gpu_agent, &agent);
   if(err == HSA_STATUS_INFO_BREAK) { err = HSA_STATUS_SUCCESS; }
   check(Getting a gpu agent, err);

   /*
	* Query the name of the agent.
	*/
   char name[64] = { 0 };
   err = hsa_agent_get_info(agent, HSA_AGENT_INFO_NAME, name);
   check(Querying the agent name, err);
   printf("The agent name is %s.\n", name);

   /*
	* Query the maximum size of the queue.
	*/
   uint32_t queue_size = 0;
   err = hsa_agent_get_info(agent, HSA_AGENT_INFO_QUEUE_MAX_SIZE, &queue_size);
   check(Querying the agent maximum queue size, err);
   printf("The maximum queue size is %u.\n", (unsigned int) queue_size);

   /*
	* Create a queue using the maximum size.
	*/
   hsa_queue_t* queue; 
   err = hsa_queue_create(agent, queue_size, HSA_QUEUE_TYPE_SINGLE, NULL, NULL, UINT32_MAX, UINT32_MAX, &queue);
   check(Creating the queue, err);

   /*
	* Load the BRIG binary.
	*/
   hsa_ext_module_t module;
   load_module_from_file("NBody.brig",&module);
   /*
	* Create hsa program.
	*/
   hsa_ext_program_t program;
   memset(&program,0,sizeof(hsa_ext_program_t));
   err = hsa_ext_program_create(HSA_MACHINE_MODEL_LARGE, HSA_PROFILE_FULL, HSA_DEFAULT_FLOAT_ROUNDING_MODE_DEFAULT, NULL, &program);
   check(Create the program, err);

   /*
	* Add the BRIG module to hsa program.
	*/
   err = hsa_ext_program_add_module(program, module);
   check(Adding the brig module to the program, err);

   /*
	* Determine the agents ISA.
	*/
   hsa_isa_t isa;
   err = hsa_agent_get_info(agent, HSA_AGENT_INFO_ISA, &isa);
   check(Query the agents isa, err);

   /*
	* Finalize the program and extract the code object.
	*/
   hsa_ext_control_directives_t control_directives;
   memset(&control_directives, 0, sizeof(hsa_ext_control_directives_t));
   hsa_code_object_t code_object;
   err = hsa_ext_program_finalize(program, isa, 0, control_directives, "", HSA_CODE_OBJECT_TYPE_PROGRAM, &code_object);
   check(Finalizing the program, err);

   /*
	* Destroy the program, it is no longer needed.
	*/
   err=hsa_ext_program_destroy(program);
   check(Destroying the program, err);

   /*
	* Create the empty executable.
	*/
   hsa_executable_t executable;
   err = hsa_executable_create(HSA_PROFILE_FULL, HSA_EXECUTABLE_STATE_UNFROZEN, "", &executable);
   check(Create the executable, err);

   /*
	* Load the code object.
	*/
   err = hsa_executable_load_code_object(executable, agent, code_object, "");
   check(Loading the code object, err);

   /*
	* Freeze the executable; it can now be queried for symbols.
	*/
   err = hsa_executable_freeze(executable, "");
   check(Freeze the executable, err);

   /*
	* Extract the symbol from the executable.
	*/
   hsa_executable_symbol_t symbol;
   err = hsa_executable_get_symbol(executable, NULL, "&__OpenCL_nbody_sim_kernel", agent, 0, &symbol);
   check(Extract the symbol from the executable, err);

   /*
	* Extract dispatch information from the symbol
	*/
   uint64_t kernel_object;
   uint32_t kernarg_segment_size;
   uint32_t group_segment_size;
   uint32_t private_segment_size;
   err = hsa_executable_symbol_get_info(symbol, HSA_EXECUTABLE_SYMBOL_INFO_KERNEL_OBJECT, &kernel_object);
   check(Extracting the symbol from the executable, err);
   err = hsa_executable_symbol_get_info(symbol, HSA_EXECUTABLE_SYMBOL_INFO_KERNEL_KERNARG_SEGMENT_SIZE, &kernarg_segment_size);
   check(Extracting the kernarg segment size from the executable, err);
   err = hsa_executable_symbol_get_info(symbol, HSA_EXECUTABLE_SYMBOL_INFO_KERNEL_GROUP_SEGMENT_SIZE, &group_segment_size);
   check(Extracting the group segment size from the executable, err);
   err = hsa_executable_symbol_get_info(symbol, HSA_EXECUTABLE_SYMBOL_INFO_KERNEL_PRIVATE_SEGMENT_SIZE, &private_segment_size);
   check(Extracting the private segment from the executable, err);

   /*
	* Create a signal to wait for the dispatch to finish.
	*/ 
   hsa_signal_t signal;
   err=hsa_signal_create(1, 0, NULL, &signal);
   check(Creating a HSA signal, err);

   /*
	* Allocate and initialize the kernel arguments and data.
	*/

	  //float4 *pos,*vel;
	  pos = (float4 *)malloc(sizeof(float4)*10);
	  vel = (float4 *)malloc(sizeof(float4)*10);
	  nbody_init(pos,vel);
	  int i;
	  /*for(i=0;i<10;i++)
		{
		printf("posion %d :%f\t%f\t%f, %f ,",i+1,pos[i].x,pos[i].y,pos[i].z,pos[i].w);
		printf("vel :%f\t%f\t%f\n",vel[i].x,vel[i].y,vel[i].z);
		}*/

	  int *num;
	  num = (int *)malloc(sizeof(int));
	  *num = 10;
	  float *del,*eps;
	  del = (float *)malloc(sizeof(float));
	  *del = 0.005;
	  eps = (float *)malloc(sizeof(float));
	  *eps = 500;
	  float4 *pos_new,*vel_new,*pblock;
	  pos_new = (float4 *)malloc(sizeof(float4)*10);
	  vel_new = (float4 *)malloc(sizeof(float4)*10);
	  pblock = (float4 *)malloc(sizeof(float4)*10);
	  for(i=0;i<10;i++){
		 pblock[i].x = 0;
		 pblock[i].y = 0;
		 pblock[i].z = 0;
		 pblock[i].w = 0;
	  }
	  for(i=0;i<10;i++){
		 pos_new[i].x = 0;
		 pos_new[i].y = 0;
		 pos_new[i].z = 0;
		 pos_new[i].w = 0;
		 vel_new[i].x = 0;
		 vel_new[i].y = 0;
		 vel_new[i].z = 0;
		 vel_new[i].w = 0;
	  }
	  /*for(i=0;i<10;i++)
		{
		printf("posion %d :%f\t%f\t%f, %f ,",i+1,pos_new[i].x,pos_new[i].y,pos_new[i].z,pos_new[i].w);
		printf("vel :%f\t%f\t%f\n",vel_new[i].x,vel_new[i].y,vel_new[i].z);
		}*/

	  err = hsa_memory_register(num,sizeof(int));
	  err = hsa_memory_register(del,sizeof(float));
	  err = hsa_memory_register(eps,sizeof(float));
	  err = hsa_memory_register(pos,sizeof(float4)*10);
	  err = hsa_memory_register(pos_new,sizeof(float4)*10);
	  err = hsa_memory_register(vel_new,sizeof(float4)*10);
	  err = hsa_memory_register(vel,sizeof(float4)*10);
	  err = hsa_memory_register(pblock,sizeof(float4)*10);

for(;;){
	  for(i=0;i<10;i++)
	  {
		 printf("Old posion %d :%f\t%f\t%f, %f",i+1,pos[i].x,pos[i].y,pos[i].z,pos[i].w);
		 printf("vel :%f\t%f\t%f\n\n",vel[i].x,vel[i].y,vel[i].z);
	  }
	  struct __attribute__ ((aligned(16))) args_t {
		 uint64_t global_offset_0;
		 uint64_t global_offset_1;
		 uint64_t global_offset_2;
		 uint64_t printf_buffer;
		 uint64_t vqueue_pointer;
		 uint64_t aqlwrap_pointer;
		 void* pos;
		 void* vel;
		 void* num;
		 void* del;
		 void* eps;
		 void* newPosition;
		 void* newVelocity;
	  } args;
	  memset(&args, 0, sizeof(args));
	  args.del = del;
	  args.eps = eps;
	  args.pos = pos;
	  args.newPosition = pos_new;
	  args.newVelocity = vel_new;
	  args.vel = vel;
	  args.num = num;

	  /*
	   * Find a memory region that supports kernel arguments.
	   */
	  hsa_region_t kernarg_region;
	  kernarg_region.handle=(uint64_t)-1;
	  hsa_agent_iterate_regions(agent, get_kernarg_memory_region, &kernarg_region);
	  err = (kernarg_region.handle == (uint64_t)-1) ? HSA_STATUS_ERROR : HSA_STATUS_SUCCESS;
	  check(Finding a kernarg memory region, err);
	  void* kernarg_address = NULL;

	  /*
	   * Allocate the kernel argument buffer from the correct region.
	   */   
	  err = hsa_memory_allocate(kernarg_region, kernarg_segment_size, &kernarg_address);
	  check(Allocating kernel argument memory buffer, err);
	  memcpy(kernarg_address, &args, sizeof(args));

	  /*
	   * Obtain the current queue write index.
	   */
	  uint64_t index = hsa_queue_load_write_index_relaxed(queue);

	  /*
	   * Write the aql packet at the calculated queue index address.
	   */
	  const uint32_t queueMask = queue->size - 1;
	  hsa_kernel_dispatch_packet_t* dispatch_packet = &(((hsa_kernel_dispatch_packet_t*)(queue->base_address))[index&queueMask]);

	  dispatch_packet->header |= HSA_FENCE_SCOPE_SYSTEM << HSA_PACKET_HEADER_ACQUIRE_FENCE_SCOPE;
	  dispatch_packet->header |= HSA_FENCE_SCOPE_SYSTEM << HSA_PACKET_HEADER_RELEASE_FENCE_SCOPE;
	  dispatch_packet->setup  |= 1 << HSA_KERNEL_DISPATCH_PACKET_SETUP_DIMENSIONS;
	  dispatch_packet->workgroup_size_x = (uint16_t)LOCAL_SIZE;
	  dispatch_packet->workgroup_size_y = (uint16_t)1;
	  dispatch_packet->workgroup_size_z = (uint16_t)1;
	  dispatch_packet->grid_size_x = (uint32_t) (GLOBAL_SIZE);
	  dispatch_packet->grid_size_y = 1;
	  dispatch_packet->grid_size_z = 1;
	  dispatch_packet->completion_signal = signal;
	  dispatch_packet->kernel_object = kernel_object;
	  dispatch_packet->kernarg_address = (void*) kernarg_address;
	  dispatch_packet->private_segment_size = private_segment_size;
	  dispatch_packet->group_segment_size = group_segment_size;
	  __atomic_store_n((uint8_t*)(&dispatch_packet->header), (uint8_t)HSA_PACKET_TYPE_KERNEL_DISPATCH, __ATOMIC_RELEASE);

	  /*
	   * Increment the write index and ring the doorbell to dispatch the kernel.
	   */
	  hsa_queue_store_write_index_relaxed(queue, index+1);
	  hsa_signal_store_relaxed(queue->doorbell_signal, index);
	  check(Dispatching the kernel, err);

	  /*
	   * Wait on the dispatch completion signal until the kernel is finished.
	   */
	  nBodyCPUReference(pos,vel,newPos,newVel);
	  
	  hsa_signal_value_t value = hsa_signal_wait_acquire(signal, HSA_SIGNAL_CONDITION_LT, 1, UINT64_MAX, HSA_WAIT_STATE_BLOCKED);
	
	
	//*************************************************************Display GL***********************************************************************************************************//
	glutInit(&argc, argv);
    glutInitWindowPosition(100,10);
    glutInitWindowSize(600,600);
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE );
    glutCreateWindow("N-body simulation");
    init();
    glutDisplayFunc(displayfunc);
    glutReshapeFunc(reShape);
    glutIdleFunc(idle);
    //glutKeyboardFunc(keyboardFunc);
	glutMainLoop();

	
	
	//*************************************************************Display GL***********************************************************************************************************//

	  for(i=0;i<10;i++)
	  {
		 printf("New posion %d :%f\t%f\t%f, %f ,",i+1,pos_new[i].x,pos_new[i].y,pos_new[i].z,pos_new[i].w);
		 printf("vel :%f\t%f\t%f\n",vel_new[i].x,vel_new[i].y,vel_new[i].z);
	  }
	  for(i=0;i<10;i++)
	  {
		 printf("New posion %d :%f\t%f\t%f, %f ,",i+1,newPos[i].x,newPos[i].y,newPos[i].z,newPos[i].w);
		 printf("vel :%f\t%f\t%f\n",newVel[i].x,newVel[i].y,newVel[i].z);
	  }
	  printf("loop:%d\n",++loopcounter);
	sleep(1);
	}
   /*
	* Cleanup all allocated resources.
	*/
   err=hsa_signal_destroy(signal);
   check(Destroying the signal, err);

   err=hsa_executable_destroy(executable);
   check(Destroying the executable, err);

   err=hsa_code_object_destroy(code_object);
   check(Destroying the code object, err);

   err=hsa_queue_destroy(queue);
   check(Destroying the queue, err);

   err=hsa_shut_down();
   check(Shutting down the runtime, err);

   //free(in);
   //free(out);

   return 0;
}
