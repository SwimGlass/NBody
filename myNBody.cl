__kernel void nbody_kern(
		float dt1,float eps,
		__global float *old_x,__global float *old_y,__global float *old_z,
		__global float *new_x,__global float *new_y,__global float *new_z,
		__global float *mass,
		__global float *vx,__global float *vy,__global float *vz,
		__local float *tmp_x,__local float *tmp_y,__local float *tmp_z,
		
		
		)
