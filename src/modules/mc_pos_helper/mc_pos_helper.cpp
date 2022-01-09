
 /****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>//0102
 */
//#include <px4_posix.h>
//#include "params.hpp"
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
//#include <px4_tasks.h>
//#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <poll.h>
//#include <platforms/px4_defines.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
//#include <px4_config.h>
//#include <nuttx/sched.h>

//#include <systemlib/systemlib.h>
#include <systemlib/err.h>
//#include <lib/geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/pos_helper.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/iusl_para.h>
#include <uORB/topics/parameter_update.h>

//
#include <drivers/drv_hrt.h>

//using namespace math;
using namespace matrix;

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

//int pos_helper_sub;
//int control_state_sub;
//int vehicle_angular_velocity_sub;
//int vehicle_attitude_sub;
int vehicle_local_position_sub;

static float pos_esti_x, pos_esti_y,pos_esti_z;
static float pos_deri_esti_x, pos_deri_esti_y, pos_deri_esti_z;
static float vel_esti_x, vel_esti_y,vel_esti_z;
static float vel_deri_esti_x, vel_deri_esti_y, vel_deri_esti_z;
static float delta_esti_x, delta_esti_y,delta_esti_z;
static float delta_deri_esti_x, delta_deri_esti_y, delta_deri_esti_z;

static float w_esti_r,w_esti_p,w_esti_y;
static float w_deri_esti_r,w_deri_esti_p,w_deri_esti_y;
static float delta_esti_wr,delta_esti_wp,delta_esti_wy;
static float delta_deri_esti_wr,delta_deri_esti_wp,delta_deri_esti_wy;



hrt_abstime now, _last_time;
float dt;
float psi_d;
float hover_thrust;
float takeoff;
//static int i=0.0f;
//static int k=1.0f;

const float _dt_min = 0.00001f;
const float _dt_max = 0.02f;
const float _lim_thr_max=20.0f;
const float _hover_thrust=15.0f;///< Thrust [0.1, 0.9] with which the vehicle hovers not accelerating down or up with level orientation
//const float _hover_thrust=11.52f;
const float _lim_tilt=0.20f;
const float _lim_thr_min=0.5f; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
//const float _lim_thr_max=-0.1f; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
//30
//const float lim_thr_min_unit=-0.9f; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
//float lim_thr_max; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1

matrix::Vector3f s_v;
matrix::Vector3f v_r;
matrix::Vector3f v_r_prev;
matrix::Vector3f v_r_deri;
matrix::Vector3f pos_iusl;
matrix::Vector3f pos_prev_iusl;
matrix::Vector3f pos_deri_iusl;
matrix::Vector3f pos_e_iusl;
//math::Vector<3> pos_e_prev_iusl;
matrix::Vector3f pos_desi_iusl;
matrix::Vector3f pos_desi_prev_iusl;
matrix::Vector3f pos_desi_deri_iusl;
matrix::Vector3f pos_e_sum_iusl{0.0f,0.0f,0.0f};
matrix::Vector3f w_r;
matrix::Vector3f w_r_prev;
matrix::Vector3f w_r_deri;
matrix::Vector3f g;
matrix::Vector3f f_iusl;  //output 1
matrix::Vector3f f_iusl_pre;
matrix::Vector3f tau;  //output 2
matrix::Vector3f v_r1;

matrix::Matrix<float,3, 3> lambda_p;
matrix::Matrix<float,3, 3> lambda_q;
matrix::Matrix<float,3, 3> K_v;
matrix::Matrix<float,3, 3> I_b;
matrix::Matrix<float,3, 3> I_b_inve;
matrix::Matrix<float,3, 3> K_w;
//math::Vector<3> tau;  //output 2


uORB::Publication<pos_helper_s>	pos_helper_pub{ORB_ID(pos_helper)};

struct pos_helper_s pos_helper {};
//struct control_state_s control_state_iusl {};
struct vehicle_angular_velocity_s vehicle_angular_velocity {};
struct vehicle_attitude_s vehicle_attitude {};
struct vehicle_local_position_s vehicle_local_position {};
struct actuator_controls_s actuator_controls {};
struct iusl_para_s iusl_param;
//struct pos_iusl
//{
//	float x;
//	float y;
//	float z;
//};
//struct position_iusl_s pos_desi_iusl;

#define G	9.8066f
//#define Mb	1.152f
#define Mb	1.50f
#define SUM_mi	0.00f
#define SUM_max_x 10.0f
#define SUM_max_y 10.0f
#define SUM_max_z 25.0f
//#define SUM_mi	0.29f
//#define I_b	9.8066f
#define wp_1	8.0f
#define wp_2	8.0f
#define wp_3	8.0f
#define wa_1	20.0f
#define wa_2	20.0f
#define wa_3	20.0f
#define k_q	    0.5f
//#define FLT_EPSILON 1.1920e-7F
#define CONSTANTS_ONE_G 9.8066f
#define Pi 3.1415926f
/**
 * daemon management function.
 */


extern "C" __EXPORT int mc_pos_helper_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int mc_pos_helper_thread_main(int argc, char *argv[]);

void fn_pos_eso(float x,float y,float z,float ux,float uy,float uz);
void fn_att_eso(float roll_rate,float pitch_rate,float yaw_rate,float u_r,float u_p,float u_y);
void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle);
void limitThrust(Vector3f &_acc_sp,Vector3f &_thr_sp);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);




static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int mc_pos_helper_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {

			//warnx("daemon already running\n");

			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("mc_pos_helper",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
                          mc_pos_helper_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

			         //   int local_positon_sub1=orb_subscribe(ORB_ID(vehicle_local_position));
			        //    struct vehicle_local_position_s vehicle_local_position1;
						while(1)
						{
                           // uORB::Subscription pos_helper_subx{ORB_ID(pos_helper)};
                           // pos_helper_s		_pos_helperx{};
                           // memset(&_pos_helperx, 0, sizeof(_pos_helperx));
                          //  pos_helper_subx.copy(&_pos_helperx);
                           //
							warnx("============press CTRL+C to abort============");

							char h_c;
							struct pollfd fds;
							int ret;
							fds.fd=0;
							fds.events=POLLIN;
							ret=poll(&fds,1,0);
							if(ret>0)
							{
                                ssize_t sr=read(0,&h_c,1);
                                 //r=0;
                                if(sr==0)
                                {};
								if(h_c==0x03||h_c==0x63||h_c=='q')
                                {

                                    //warnx("User abort\n");
									break;
								}
							}
							usleep(800000);		//500ms
						}

						return 0;

			/*
			 * struct sensor_combined_s h_sensor;
	           //memset(&sensor, 0, sizeof(sensor));
	         *
			 */
		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

void fn_pos_eso(float x,float y,float z,float ux,float uy,float uz)
{
	delta_deri_esti_x=wp_1*wp_1*wp_1*(x-pos_esti_x);
	delta_esti_x=delta_esti_x+delta_deri_esti_x*dt;
	vel_deri_esti_x=ux+delta_esti_x+3*wp_1*wp_1*(x-pos_esti_x);
	vel_esti_x=vel_esti_x+vel_deri_esti_x*dt;
	pos_deri_esti_x=vel_esti_x+3*wp_1*(x-pos_esti_x);
	pos_esti_x=pos_esti_x+pos_deri_esti_x*dt;
    //warnx("vel_deri_esti_x   %f ux %f", double(vel_deri_esti_x),double(ux));

	delta_deri_esti_y=wp_2*wp_2*wp_2*(y-pos_esti_y);
	delta_esti_y=delta_esti_y+delta_deri_esti_y*dt;
	vel_deri_esti_y=uy+delta_esti_y+3*wp_2*wp_2*(y-pos_esti_y);
	vel_esti_y=vel_esti_y+vel_deri_esti_y*dt;
	pos_deri_esti_y=vel_esti_y+3*wp_2*(y-pos_esti_y);
	pos_esti_y=pos_esti_y+pos_deri_esti_y*dt;
 //  warnx("pos_deri_esti_y   %f  uy%f", double(pos_deri_esti_y),double(uy));

	delta_deri_esti_z=wp_3*wp_3*wp_3*(z-pos_esti_z);
	delta_esti_z=delta_esti_z+delta_deri_esti_z*dt;
	vel_deri_esti_z=uz+delta_esti_z+3*wp_3*wp_3*(z-pos_esti_z);
	vel_esti_z=vel_esti_z+vel_deri_esti_z*dt;
	pos_deri_esti_z=vel_esti_z+3*wp_3*(z-pos_esti_z);
	pos_esti_z=pos_esti_z+pos_deri_esti_z*dt;
  //  warnx("vel_esti_z   %f  localpos z=%f", double(vel_esti_z),double(z));
}

void fn_att_eso(float roll_rate,float pitch_rate,float yaw_rate,float u_r,float u_p,float u_y)
{
	delta_deri_esti_wr=wa_1*wa_1*(roll_rate-w_esti_r);
	delta_esti_wr=delta_esti_wr+delta_deri_esti_wr*dt;
	w_deri_esti_r=u_r+delta_deri_esti_wr+2*wa_1*(roll_rate-w_esti_r);
	w_esti_r=w_esti_r+w_deri_esti_r*dt;

	delta_deri_esti_wp=wa_2*wa_2*(pitch_rate-w_esti_p);
	delta_esti_wp=delta_esti_wp+delta_deri_esti_wp*dt;
	w_deri_esti_p=u_p+delta_deri_esti_wp+2*wa_2*(pitch_rate-w_esti_p);
	w_esti_p=w_esti_p+w_deri_esti_p*dt;

    delta_deri_esti_wy=wa_3*wa_3*(yaw_rate-w_esti_y);
	delta_esti_wy=delta_esti_wy+delta_deri_esti_wy*dt;
    w_deri_esti_y=u_y+delta_deri_esti_wy+2*wa_3*(yaw_rate-w_esti_y);
	w_esti_y=w_esti_y+w_deri_esti_y*dt;

}

void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle)
{
    // determine tilt
    const float dot_product_unit = body_unit.dot(world_unit);
    float angle = acosf(dot_product_unit);
    // limit tilt
    angle = math::min(angle, max_angle);
    Vector3f rejection = body_unit - (dot_product_unit * world_unit);

    // corner case exactly parallel vectors
    if (rejection.norm_squared() < FLT_EPSILON) {
        rejection(0) = 1.f;
    }

    body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.unit();
}


void limitThrust(Vector3f &_acc_sp,Vector3f &_thr_sp)
{
    // Assume standard acceleration due to gravity in vertical direction for attitude generation
    Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
    limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
    // Scale thrust assuming hover thrust produces standard gravity
    float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
    // Project thrust to planned body attitude
    collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
    collective_thrust = math::min(collective_thrust, -_lim_thr_min);
    _thr_sp = body_z * collective_thrust;
}
//float * cross_product(float ax,float ay,float az,float bx,float by,float bz)
//{
//float v[3];
//v[0]=ay*bz-az*by;
//v[1]=az*bx-ax*bz;
//v[2]=ax*by-ay*bx;
//return v;
//}


int mc_pos_helper_thread_main(int argc, char *argv[])
{

    //warnx("[daemon] starting\n");
        //bool updated;
        thread_running = true;
        memset(&pos_helper, 0, sizeof(pos_helper));
        //pos_helper_sub=orb_subscribe(ORB_ID(pos_helper));
	/* limit the update rate to 5 Hz */
        //orb_set_interval(pos_helper_sub, 200);
        //memset(&control_state_iusl, 0, sizeof(control_state_iusl));
        //control_state_sub=orb_subscribe(ORB_ID(control_state));
        memset(&vehicle_angular_velocity, 0, sizeof(vehicle_angular_velocity));
        //vehicle_angular_velocity_sub=orb_subscribe(ORB_ID(vehicle_angular_velocity));
        uORB::Subscription vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
        memset(&vehicle_attitude, 0, sizeof(vehicle_attitude));
        //vehicle_attitude_sub=orb_subscribe(ORB_ID(vehicle_attitude));
        uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
        memset(&vehicle_local_position, 0, sizeof(vehicle_local_position));
       // uORB::Subscription vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
        uORB::Subscription actuator_controls_sub{ORB_ID(actuator_controls_0)};
        memset(&actuator_controls, 0, sizeof(actuator_controls));
        uORB::Subscription vehicle_local_position_sub2{ORB_ID(vehicle_local_position)};
        uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
        uORB::Subscription para_sub{ORB_ID(iusl_para)};
        uORB::Subscription para_update_sub{ORB_ID(parameter_update)};
        //uORB::Subscription hover_thrust_sub{ORB_ID(hover_thrust_estimate)};
       // #include <uORB/topics/hover_thrust_estimate.h>
       // manual_control_setpoint_s manual_control_setpoint1;
       // manual_control_setpoint1.r=0.0f;
        //uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};

	//R_d.identity();
	lambda_p.identity();
    lambda_p(0,0)=1.00f;
    lambda_p(1,1)=1.00f;
    lambda_p(2,2)=1.30f;
    //lambda_p=lambda_p*4.0f;
    //lambda_p=lambda_p*0.1f;



	K_v.identity();  
    K_v(0,0)=0.5f;
    K_v(1,1)=0.5f;
    K_v(2,2)=1.2f;
    //K_v=K_v*5.0f;

    matrix::Matrix<float,3, 3> M_deri1;
    matrix::Matrix<float,3, 3> M_deri2;
    matrix::Matrix<float,3, 3> M_deri3;
    //matrix::Matrix<float,3, 3> M_deri4;
    M_deri1.identity();
    M_deri1(0,0)=1.0f;
    M_deri1(1,1)=1.0f;
    M_deri1(2,2)=1.0f;
    M_deri2.identity();
    M_deri2(0,0)=100.00f;
    M_deri2(1,1)=100.00f;
    M_deri2(2,2)=100.00f;
    M_deri3.identity();
    M_deri3(0,0)=1.0f;
    M_deri3(1,1)=1.0f;
    M_deri3(2,2)=1.0f;

	g(0)=0.0f;
	g(1)=0.0f;
	g(2)=G;

    Vector3f pos_desi_temp(0.673f,1.137f,-0.08f);

    pos_iusl(0)=0.0f;
    pos_iusl(1)=0.0f;
    pos_iusl(2)=0.0f;

    pos_deri_iusl(0)=0.0f;
    pos_deri_iusl(1)=0.0f;
    pos_deri_iusl(2)=0.0f;

    pos_desi_deri_iusl(0)=0.0f;
    pos_desi_deri_iusl(1)=0.0f;
    pos_desi_deri_iusl(2)=0.0f;

    pos_e_sum_iusl(0)=0.0f;
    pos_e_sum_iusl(1)=0.0f;
    pos_e_sum_iusl(2)=0.0f;

    takeoff=0.0f;




	/* one could wait for multiple topics with this technique, just using one here */
	//px4_pollfd_struct_t fds[1] = {
	//{ .fd =pos_helper_sub,   .events = POLLIN },
	//		/* there could be more file descriptors here, in the form like:
	//		 * { .fd = other_sub_fd,   .events = POLLIN },
	//		 */
	//	};

    vehicle_local_position_sub= orb_subscribe(ORB_ID(vehicle_local_position));
    orb_set_interval(vehicle_local_position_sub, 10);

    px4_pollfd_struct_t fds[1];
    fds[0].fd = vehicle_local_position_sub;
	fds[0].events = POLLIN;

    int error_counter = 0;
    vehicle_local_position_sub2.copy(&vehicle_local_position);
    psi_d=vehicle_local_position.heading;

	while (!thread_should_exit) {
      //  warnx("[daemon] starting 1\n");

		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
                                int poll_ret = px4_poll(fds, 1, 150);
				/* handle the poll result */
				if (poll_ret == 0) {
					/* this means none of our providers is giving us data */
					PX4_ERR("Got no data within a second");

				} else if (poll_ret < 0) {
					/* this is seriously bad - should be an emergency */
					if (error_counter < 10 || error_counter % 50 == 0) {
						/* use a counter to prevent flooding (and slowing us down) */
						PX4_ERR("ERROR return value from poll(): %d", poll_ret);
					}

					error_counter++;

				} else {
 //                   warnx("[daemon] starting  2\n");

					if (fds[0].revents & POLLIN) {

                        if (para_update_sub.updated()) {
                            // clear update

                            para_sub.copy(&iusl_param);
                            lambda_p(0,0)=iusl_param.lambda_p_x;
                            lambda_p(1,1)=iusl_param.lambda_p_y;
                            lambda_p(2,2)=iusl_param.lambda_p_z;

                            K_v(0,0)=iusl_param.iusl_kv_x;
                            K_v(1,1)=iusl_param.iusl_kv_y;
                            K_v(2,2)=iusl_param.iusl_kv_z;



                            M_deri1(0,0)=iusl_param.iusl_d_x;
                            M_deri1(1,1)=iusl_param.iusl_d_y;
                            M_deri1(2,2)=iusl_param.iusl_d_z;

                            M_deri2(0,0)=iusl_param.iusl_i_x;
                            M_deri2(1,1)=iusl_param.iusl_i_y;
                            M_deri2(2,2)=iusl_param.iusl_i_z;

                            M_deri3(0,0)=iusl_param.iusl_p_x;
                            M_deri3(1,1)=iusl_param.iusl_p_y;
                            M_deri3(2,2)=iusl_param.iusl_p_z;


                            //warnx("lambda_p   %f  ", double(lambda_p(0,0)));

                        }

                                         //    warnx("lambda_p   %f   k_v %f", double(lambda_p(0,0)),double(K_v(0,0)));
//iusl
                                                orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &vehicle_local_position);
 //                       warnx("[daemon] starting 3\n");
                                                now = hrt_absolute_time();
                                                dt = math::constrain((now - _last_time) / 1e6f, _dt_min, _dt_max);
                                                _last_time = now;
                                                //f_iusl_pre=f_iusl;
                                            /* obtained data for the first file descriptor */
                                             //   orb_copy(ORB_ID(vehicle_local_position),vehicle_local_position_sub, &vehicle_local_position);
                                              //  i=i+k*0.0015f;

                                                vehicle_attitude_sub.copy(&vehicle_attitude);
                                               // warnx("localpos x  %f z %f", double(vehicle_local_position.x),double(vehicle_local_position.z));

                                               // orb_copy(ORB_ID(actuator_controls),actuator_controls_sub, &actuator_controls);
                                                actuator_controls_sub.copy(&actuator_controls);
                                                float f_temp= actuator_controls.control[actuator_controls_s::INDEX_THROTTLE];
                                                //warnx("f_temp  %f  ", double(f_temp));
                                                //f_temp=(f_temp*3200.0f*0.01805f-20.89f)*4.0f;
                                                f_temp=f_temp*20.0f;
                                                //warnx("f_temp_abs  %f  ", double(f_temp));
                                                Vector3f fv_temp(0.0f,0.0f,f_temp);
                                                matrix::Quatf q_att(vehicle_attitude.q);
                                                //warnx("q_att   %f %f %f %f", double(vehicle_attitude.q[0]),double(vehicle_attitude.q[1]),double(vehicle_attitude.q[2]),double(vehicle_attitude.q[3]));
                                                matrix::Dcmf _R(q_att);
                                               // _R=_R.transpose();
                                             //   matrix::Eulerf eulertest(q_att);
                                             //   Dcmf _R1=Eulerf(float(eulertest(0)),float(eulertest(1)),float(eulertest(2)));

                                                fv_temp=_R*fv_temp;
                                                //warnx("fv_temp  %f  %f  %f", double(fv_temp(0)),double(fv_temp(1)),double(fv_temp(2)));
//						orb_copy(ORB_ID(position_desire_iusl),position_desire_iusl_sub, &pos_desi_iusl);



                                                uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
                                                vehicle_status_s vehicle_status{};
                                                vehicle_status_sub.copy(&vehicle_status);
                                                if((vehicle_status.nav_state==vehicle_status_s::NAVIGATION_STATE_ACRO)&&(vehicle_status.arming_state==vehicle_status_s::ARMING_STATE_ARMED))
                                                {
                                              //  Vector3f pos_desi_temp1(0.673f, 1.137f, -0.8f);
                                                    manual_control_setpoint_s manual_control_setpoint;

                                                   if (_manual_control_setpoint_sub.update(&manual_control_setpoint)){
                                                      //  PX4_WARN("manual_control_setpoint.2  %f %f %f %f", double(manual_control_setpoint.x),double(manual_control_setpoint.y),double(manual_control_setpoint.z),double(manual_control_setpoint.r));
                                                        if((fabsf((manual_control_setpoint.x))<0.1f)){
                                                             manual_control_setpoint.x=0.0f;
                                                        }
                                                        if((fabsf((manual_control_setpoint.y))<0.1f)){
                                                             manual_control_setpoint.y=0.0f;
                                                        }
                                                        if((fabsf((manual_control_setpoint.z-0.50f))<0.1f)){
                                                             manual_control_setpoint.z=0.50f;
                                                        }
                                                        if((fabsf((manual_control_setpoint.r))<0.1f)){
                                                             manual_control_setpoint.r=0.0f;
                                                        }
                                                        if(vehicle_local_position.z>-0.5f)
                                                        {
                                                               takeoff=0.0f;
                                                        }else{
                                                            takeoff=1.0f;
                                                        }
                                                        //PX4_WARN("manual_control_setpoint.2  %f %f %f %f", double(manual_control_setpoint.x),double(manual_control_setpoint.y),double(manual_control_setpoint.z),double(manual_control_setpoint.r));
                                                        pos_desi_temp(0)=pos_desi_temp(0)+3.0f*takeoff*manual_control_setpoint.x*dt;//0.01*manual_control_setpoint.x
                                                        pos_desi_temp(1)=pos_desi_temp(1)+3.0f*takeoff*manual_control_setpoint.y*dt;
                                                        pos_desi_temp(2)=pos_desi_temp(2)-3.0f*(manual_control_setpoint.z-0.50f)*dt;
                                                       // PX4_WARN("pos_desi_temp(2) %f",double(pos_desi_temp(2)));
                                                        //manual_control_setpoint1.r=manual_control_setpoint.r;
                                                        if(pos_desi_temp(2)>0.3f)
                                                        {
                                                            pos_desi_temp(2)=0.3f;
                                                        }
                                                        if(pos_desi_temp(2)<-2.0f)
                                                        {
                                                            pos_desi_temp(2)=-2.0f;
                                                        }

                                                        psi_d=psi_d+0.01f*manual_control_setpoint.r*dt;
                                                        while(psi_d>Pi){
                                                            psi_d=psi_d-2*Pi;
                                                        }
                                                        while(psi_d<(-Pi)){
                                                            psi_d=psi_d+2*Pi;
                                                        }
                                                    }
                                                }else{
                                                    pos_e_sum_iusl(0)=0.0f;
                                                    pos_e_sum_iusl(1)=0.0f;
                                                    pos_e_sum_iusl(2)=0.0f;
                                                    psi_d=vehicle_local_position.heading;
                                                    pos_desi_temp(0)=vehicle_local_position.x;
                                                    pos_desi_temp(1)=vehicle_local_position.y;
                                                    pos_desi_temp(2)=vehicle_local_position.z;
                                                }
                                               // PX4_WARN("pos_desi_temp2  %f %f %f", double(pos_desi_temp(0)),double(pos_desi_temp(1)),double(pos_desi_temp(2)));
                                                pos_prev_iusl=pos_iusl;
                                                pos_desi_iusl=pos_desi_temp;
                                                //pos_desi_iusl(0)=3.0f*sin(i);
                                                pos_iusl(0)=vehicle_local_position.x;
                                                pos_iusl(1)=vehicle_local_position.y;
                                                pos_iusl(2)=vehicle_local_position.z;
                                               // pos_e_iusl=pos_iusl-pos_desi_iusl;
                                                //pos_e_iusl(0)=pos_iusl(0)-pos_desi_iusl(0);
                                              //  pos_e_iusl(1)=pos_iusl(1)-pos_desi_iusl(1);
                                         //        ccccccccccccccccccccc

                                              //  pos_e_iusl=pos_desi_iusl-pos_iusl;
                                               pos_e_iusl=pos_iusl-pos_desi_iusl;

                                         //    cccccccccccccccccccccccccccccc

                        pos_e_sum_iusl=pos_e_sum_iusl+pos_e_iusl*dt;
                        //float pos_e_sum_iusl_temp = math::min(fabsf(pos_e_sum_iusl(2)), SUM_max) ;
                       // pos_e_sum_iusl=pos_e_sum_iusl*(pos_e_sum_iusl_temp/fabsf(pos_e_sum_iusl(2)));
                        if((fabsf((pos_e_sum_iusl(0)))>SUM_max_x)){
                                                     pos_e_sum_iusl(0)=sign(pos_e_sum_iusl(0))*SUM_max_x;
                                                }else{

                                                    pos_e_sum_iusl(0)=pos_e_sum_iusl(0);
                                                }

                                                if((fabsf((pos_e_sum_iusl(1)))>SUM_max_y)){
                                                     pos_e_sum_iusl(1)=sign(pos_e_sum_iusl(1))*SUM_max_y;
                                                }else{

                                                    pos_e_sum_iusl(1)=pos_e_sum_iusl(1);
                                                }
                                                if((fabsf((pos_e_sum_iusl(2)))>SUM_max_z)){
                                                     pos_e_sum_iusl(2)=sign(pos_e_sum_iusl(2))*SUM_max_z;
                                                }else{

                                                    pos_e_sum_iusl(2)=pos_e_sum_iusl(2);
                                                }
                        //warnx("pos_e_sum_iusl   %f %f %f ", double(pos_e_sum_iusl(0)),double(pos_e_sum_iusl(1)),double(pos_e_sum_iusl(2)));
						/* copy  data into local buffer */
                                                 //t = hrt_absolute_time();
                                                //dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
                                                //dt = fmaxf(fminf(0.02, dt), 0.0002);		// constrain dt from 0.2 to 20 ms
                                                //t_prev = t;

                        //pos_deri_iusl=(pos_iusl-pos_prev_iusl)/dt;
                        pos_deri_iusl(0)=vehicle_local_position.vx;
                        pos_deri_iusl(1)=vehicle_local_position.vy;
                        pos_deri_iusl(2)=vehicle_local_position.vz;
                       // pos_desi_deri_iusl=(pos_desi_iusl-pos_desi_prev_iusl)/dt;
                        pos_desi_deri_iusl(0)=0.0f;
                        pos_desi_deri_iusl(1)=0.0f;
                        pos_desi_deri_iusl(2)=0.0f;
                        //pos_e_deri_iusl.y=(pos_e_iusl.y-pos_e_prev_iusl.y)/dt;
                        //pos_e_deri_iusl.z=(pos_e_iusl.z-pos_e_prev_iusl.z)/dt;
                        //pos_deri_iusl=(pos_iusl-pos_pre_iusl)/dt;
                        //pos_deri_iusl.y=(pos_iusl.y-pos_prev_iusl.y)/dt;
                        //pos_deri_iusl.z=(pos_iusl.z-pos_prev_iusl.z)/dt;
						/* set att and publish this information for other apps
        make px4_sitl_default gazebo				 the following does not have any meaning, it's just an example
						*/
                       // s_v=pos_e_deri_iusl+2*lambda_p*pos_e_iusl+lambda_p*lambda_p*pos_e_sum_iusl;
//    ccccccccccccccccccccccccc

                        v_r_prev=v_r1;


                        v_r=(pos_desi_deri_iusl)*1.0f-(lambda_p*pos_e_iusl)*2.0f-(lambda_p*lambda_p*pos_e_sum_iusl);
                        //v_r_deri=(v_r1-v_r_prev)/dt;
                        v_r_deri=(-pos_desi_deri_iusl*0.0f)-(lambda_p*pos_deri_iusl)*2.0f-(lambda_p*lambda_p*pos_e_iusl);
                      //  v_r_deri=(-pos_desi_deri_iusl*0.0f)-(lambda_p*pos_deri_iusl)*2.0f-(lambda_p*lambda_p*pos_e_iusl);

                        s_v= pos_deri_iusl - v_r;
                        //s_v=M_deri1*(pos_deri_iusl*1.0f)-v_r;
 // cccccccccccccccccccccccccccc

                        pos_helper.v_r[0]=v_r(0);
                        pos_helper.v_r[1]=v_r(1);
                        pos_helper.v_r[2]=v_r(2);
//
                        pos_helper.s_v[0]=s_v(0);
                        pos_helper.s_v[1]=s_v(1);
                        pos_helper.s_v[2]=s_v(2);

                        pos_helper.pos_desi_deri[0]=pos_desi_deri_iusl(0);
                        pos_helper.pos_desi_deri[1]=pos_desi_deri_iusl(1);
                        pos_helper.pos_desi_deri[2]=pos_desi_deri_iusl(2);

                        pos_helper.pos_e_sum_iusl[0]=pos_e_sum_iusl(0);
                        pos_helper.pos_e_sum_iusl[1]=pos_e_sum_iusl(1);
                        pos_helper.pos_e_sum_iusl[2]=pos_e_sum_iusl(2);
                        //warnx("v_r   %f %f %f ", double(v_r(0)),double(v_r(1)),double(v_r(2)));

                       // pos_deri_iusl.z=(pos_iusl.z-pos_prev_iusl.z)/dt;
                        matrix::Vector3f u_v=(g*(Mb+SUM_mi)-fv_temp)/Mb;
                        //warnx("uv  %f  %f  %f", double(u_v(0)),double(u_v(1)),double(u_v(2)));

                        pos_desi_prev_iusl=pos_desi_iusl;
                        fn_pos_eso(pos_iusl(0),pos_iusl(1),pos_iusl(2),u_v(0),u_v(1),u_v(2));
                       // math::Vector<3> delta_v=fn_eso;
                        matrix::Vector3f delta_v(delta_esti_x, delta_esti_y,delta_esti_z);
                       // matrix::Vector3f delta_v(0.0f, 0.0f,0.0f);
                        //matrix::Vector3f m_0(0.0f, 0.0f,-13.2f);
                        //f_iusl=(-g*(Mb+SUM_mi))+((K_v*s_v-M_deri4*v_r_deri+delta_v)*Mb);
                       // f_iusl=(m_0)+((K_v*s_v-M_deri4*v_r_deri+delta_v)*Mb);
                        f_iusl=-((g*(Mb+SUM_mi))+((K_v*s_v-v_r_deri+delta_v*0.0f)*Mb));

                        pos_helper.thrust_sp_ori[0]=f_iusl(0);
                        pos_helper.thrust_sp_ori[1]=f_iusl(1);
                        //pos_helper.thrust_sp[2]=f_iusl_pre(2);
                        pos_helper.thrust_sp_ori[2]=f_iusl(2);

                        //warnx("pos_helper.thrust_sp yuanshi   %f %f %f", double(f_iusl(0)),double(f_iusl(1)),double(f_iusl(2)));

                     //   float thrust_sp_xyz_norm = f_iusl.norm();
                     //   f_iusl = f_iusl / thrust_sp_xyz_norm  * 20.0f;
                        Vector3f _acc_sp=f_iusl/Mb+g;
                       // warnx("pos_helper.acc_sp    %f %f %f", double(_acc_sp(0)),double(_acc_sp(1)),double(_acc_sp(2)));
                       // warnx("pos_helper.thrust_sp suofanghou   %f %f %f", double(f_iusl(0)),double(f_iusl(1)),double(f_iusl(2)));

                        limitThrust(_acc_sp,f_iusl);
                        //warnx("pos_helper.thrust_sp yueshuhou   %f %f %f", double(f_iusl(0)),double(f_iusl(1)),double(f_iusl(2)));

                        pos_helper.thrust_sp_adj[0]=f_iusl(0);
                        pos_helper.thrust_sp_adj[1]=f_iusl(1);
                        //pos_helper.thrust_sp[2]=f_iusl_pre(2);
                        pos_helper.thrust_sp_adj[2]=f_iusl(2);
                        // Saturate maximal vertical thrust
                        f_iusl(2) = math::max(f_iusl(2), -_lim_thr_max);

                        // Get allowed horizontal thrust after prioritizing vertical control
                        const float thrust_max_squared = _lim_thr_max * _lim_thr_max;
                        const float thrust_z_squared = f_iusl(2) * f_iusl(2);
                        const float thrust_max_xy_squared = thrust_max_squared - thrust_z_squared;
                        float thrust_max_xy = 0;

                        if (thrust_max_xy_squared > 0) {
                            thrust_max_xy = sqrtf(thrust_max_xy_squared);
                        }

                        // Saturate thrust in horizontal direction
                        Vector2f thrust_sp_xy(f_iusl(0),f_iusl(1));
                        float thrust_sp_xy_norm = thrust_sp_xy.norm();

                        //Vector3f _thr_sp;
                        if (thrust_sp_xy_norm > thrust_max_xy) {
                            f_iusl(0) = thrust_sp_xy(0) / thrust_sp_xy_norm * thrust_max_xy;
                            f_iusl(1) = thrust_sp_xy(1) / thrust_sp_xy_norm * thrust_max_xy;
                        }

                       //  f_iusl(0)=0.0f;
                       //  f_iusl(1)=0.0f;
                       //    f_iusl(2)=-20.0f;



                        //pos_helper.thrust_sp[0]=f_iusl(0);
                        //pos_helper.thrust_sp[1]=f_iusl(1);
                        //pos_helper.thrust_sp[2]=f_iusl(2);

                     //   if(i==0){
                                             //       f_iusl_pre(0)=f_iusl(0);
                                             //       f_iusl_pre(1)=f_iusl(1);
                                                  //  f_iusl_pre(2)=f_iusl(2);
                                             //       i=i+1;
                                             //   }else{
                                             //       i=i+1;
                                              //      if(i==60){
                                              //          i=0;
                                              //      }
                                              //  }
                        pos_helper.thrust_sp[0]=f_iusl(0);
                        pos_helper.thrust_sp[1]=f_iusl(1);
                        //pos_helper.thrust_sp[2]=f_iusl_pre(2);                        
                        pos_helper.thrust_sp[2]=f_iusl(2);


                        //test
                        manual_control_setpoint_s manual_control_setpoint1;

                      _manual_control_setpoint_sub.copy(&manual_control_setpoint1);
                        pos_helper.thrust_sp[0]=0.0f;
                        pos_helper.thrust_sp[1]=0.0f;
                        pos_helper.thrust_sp[2]=manual_control_setpoint1.z;
                      //  PX4_WARN("manual_control_setpoint.2  %f ", double(manual_control_setpoint1.z));
                        //test
                      //  pos_helper.thrust_sp[0]=0.0f;
                      //  pos_helper.thrust_sp[1]=0.0f;
                      //  pos_helper.thrust_sp[2]=14.104f;
                       // pos_helper.timestamp = hrt_absolute_time();
                      //  R_d.identity();
                        //R_d = matrix::Eulerf(phi_d,theta_d,pos_helper.psi_d);
                        //R_d = matrix::

                        //matrix::Quatf q_sp = R;
                        //roll pitch yaw
                        //orb_check(vehicle_attitude_sub, &updated);
                        //if (updated)

                        pos_helper.timestamp = hrt_absolute_time();





                        //static float delta_esti_x, delta_esti_y,delta_esti_z;
                       // static float delta_deri_esti_x, delta_deri_esti_y, delta_deri_esti_z;
                        // warnx("pos_helper.delta_v   %f %f %f ", double(pos_helper.delta_v[0]),double(pos_helper.delta_v[1]),double(pos_helper.delta_v[2]));
                           //warnx("pos_helper.pos_esti   %f %f %f  pos_real %f %f %f", double(pos_helper.pos_esti[0]),double(pos_helper.pos_esti[1]),double(pos_helper.pos_esti[2]), double(pos_iusl(0)),double(pos_iusl(1)),double(pos_iusl(2)));
                        // warnx("pos_helper.pos_deri_esti   %f %f %f", double(pos_helper.pos_deri_esti[0]),double(pos_helper.pos_deri_esti[1]),double(pos_helper.pos_deri_esti[2]));
                       //  warnx("pos_helper.vel_esti   %f %f %f  vel_real %f %f %f", double(pos_helper.vel_esti[0]),double(pos_helper.vel_esti[1]),double(pos_helper.vel_esti[2]), double(vehicle_local_position.vx),double(vehicle_local_position.vy),double(vehicle_local_position.vz));
                      //   warnx("pos_helper.vel_deri_esti   %f %f %f", double(pos_helper.vel_deri_esti[0]),double(pos_helper.vel_deri_esti[1]),double(pos_helper.vel_deri_esti[2]));
                       //  warnx("pos_helper.delta_deri_esti   %f %f %f", double(pos_helper.delta_deri_esti[0]),double(pos_helper.delta_deri_esti[1]),double(pos_helper.delta_deri_esti[2]));
                       //  warnx("pos_helper.delta_esti   %f %f %f", double(delta_esti_x),double(delta_esti_y),double(delta_esti_z));
                         //warnx("pos_helper.thrust_sp   %f %f %f", double(f_iusl(0)),double(f_iusl(1)),double(f_iusl(2)));

                      //   warnx("pos_helper.delta_deri_esti   %f %f %f ", double(delta_deri_esti_wr),double(delta_deri_esti_wp),double(delta_deri_esti_wy));
                      //   warnx("pos_helper.delta_esti   %f %f %f  ", double(delta_esti_wr),double(delta_esti_wp),double(delta_esti_wy));
                      //   warnx("pos_helper.w_deri_esti   %f %f %f", double(w_deri_esti_r),double(w_deri_esti_p),double(w_deri_esti_y));
                      //
                         //warnx("pos_helper.torque_sp   %f %f %f", double(tau(0)),double(tau(1)),double(tau(2)));



                        //test
                        pos_helper.lambda_p_x=lambda_p(0,0);
                        pos_helper.lambda_p_y=lambda_p(1,1);
                        pos_helper.lambda_p_z=lambda_p(2,2);


                        pos_helper.iusl_kv_x=K_v(0,0);
                        pos_helper.iusl_kv_y=K_v(1,1);
                        pos_helper.iusl_kv_z=K_v(2,2);



                        pos_helper.iusl_d_x=M_deri1(0,0);
                        pos_helper.iusl_d_x=M_deri1(1,1);
                        pos_helper.iusl_d_x=M_deri1(2,0);


                        pos_helper.iusl_i_x=M_deri2(0,0);
                        pos_helper.iusl_i_y=M_deri2(1,1);
                        pos_helper.iusl_i_z=M_deri2(2,2);

                        pos_helper.iusl_p_x=M_deri3(0,0);
                        pos_helper.iusl_p_y=M_deri3(1,1);
                        pos_helper.iusl_p_z=M_deri3(2,2);

                        pos_helper.iusl_kw_x=iusl_param.iusl_kw_x;
                        pos_helper.iusl_kw_y=iusl_param.iusl_kw_y;
                        pos_helper.iusl_kw_z=iusl_param.iusl_kw_z;

                        pos_helper.lambda_q_x=iusl_param.lambda_q_x;
                        pos_helper.lambda_q_y=iusl_param.lambda_q_y;
                        pos_helper.lambda_q_z=iusl_param.lambda_q_z;

                        pos_helper.pos_x_d=pos_desi_temp(0);
                        pos_helper.pos_y_d=pos_desi_temp(1);
                        pos_helper.pos_z_d=pos_desi_temp(2);

                        pos_helper.psi_d=psi_d;
                        pos_helper_pub.publish(pos_helper);


                        //math::Vector<3> pos_e_prev_iusl;


						//orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
//iusl
					}

					/* there could be more file descriptors here, in the form like:
					 * if (fds[1..n].revents & POLLIN) {}
					 */
				}

	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}

