/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "WorkItemExample.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;
using namespace matrix;


WorkItemExample::WorkItemExample() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

WorkItemExample::~WorkItemExample()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemExample::init()
{
    ScheduleOnInterval(2000_us); // 1000 us interval, 1000 Hz rate

    lambda_q.identity();
    lambda_q(0,0)=1.5f;
    lambda_q(1,1)=1.5f;
    lambda_q(2,2)=2.1f;
    //lambda_q=lambda_q*2.5f;


    I_b.identity();
    I_b(0,0)=0.0284f;
    I_b(1,1)=0.0284f;
    I_b(2,2)=0.03357f;
    I_b_inve.identity();
    I_b_inve(0,0)=35.211f;
    I_b_inve(1,1)=35.211f;
    I_b_inve(2,2)=29.788f;

  //  I_b.identity();
  //  I_b(0,0)=0.03f;
  //  I_b(1,1)=0.03f;
 //   I_b(2,2)=0.06f;
   // I_b_inve.identity();
  //  I_b_inve(0,0)=33.3333f;
  //  I_b_inve(1,1)=33.3333f;
  //  I_b_inve(2,2)=16.6666f;

    K_w.identity();
    K_w(0,0)=1.0f;
    K_w(1,1)=1.0f;
    //K_w=K_w*1.5f;
    K_w(2,2)=1.5f;

    g(0)=0.0f;
    g(1)=0.0f;
    g(2)=G;

     phi_d=0.0f;
     theta_d=0.0f;
     phi_d_pre=0.0f;
     theta_d_pre=0.0f;

     f_iusl(0)=0.0f;
     f_iusl(1)=0.0f;
     f_iusl(2)=0.0f;
	return true;
}

void WorkItemExample::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);


    // DO WORK
    if (para_update_sub.updated()) {
        // clear update
        iusl_para_s iusl_param;
        para_sub.copy(&iusl_param);
        lambda_q(0,0)=iusl_param.lambda_q_x;
        lambda_q(1,1)=iusl_param.lambda_q_y;
        lambda_q(2,2)=iusl_param.lambda_q_z;

        K_w(0,0)=iusl_param.iusl_kw_x;
        K_w(1,1)=iusl_param.iusl_kw_y;
        K_w(2,2)=iusl_param.iusl_kw_z;


        //warnx("lambda_p   %f  ", double(lambda_p(0,0)));

    }
            //    PX4_WARN("para.qz[2] %f",double(lambda_q(2,2)));
                now = hrt_absolute_time();
                dt = math::constrain((now - _last_time) / 1e6f, _dt_min, _dt_max);
                _last_time = now;
                vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
                pos_helper_sub.copy(&pos_helper);
                vehicle_attitude_sub.copy(&vehicle_attitude);
                matrix::Quatf q_att(vehicle_attitude.q);
    //warnx("q_att   %f %f %f %f", double(vehicle_attitude.q[0]),double(vehicle_attitude.q[1]),double(vehicle_attitude.q[2]),double(vehicle_attitude.q[3]));
                matrix::Dcmf _R(q_att);
                f_iusl(0)=pos_helper.thrust_sp[0];
                f_iusl(1)=pos_helper.thrust_sp[1];
                f_iusl(2)=pos_helper.thrust_sp[2];
                float f_iusl_abs=f_iusl.length();
                //pos_helper.psi_d=0.0f;
                float temp1=sinf(pos_helper.psi_d);
                float temp2=cosf(pos_helper.psi_d);
                //The value is in units of radians.
                //returned is the one between -pi/2 and pi/2 (inclusive).
                 phi_d_pre=phi_d;
                 theta_d_pre=theta_d;
                 phi_d=asinf((f_iusl(0)*temp1-f_iusl(1)*temp2)/f_iusl_abs);

                 theta_d=0.0f;


                //theta_d=0.0f;
                if((f_iusl(2)<(-0.001f))||(f_iusl(2)>(0.001f)))
               {
                        theta_d=atanf((f_iusl(0)*temp2+f_iusl(1)*temp1)/f_iusl(2));//-pai/2~pai/2
               }else{
                    theta_d=0.0f;
               }

             //  theta_d=0.0f;
             //  phi_d=0.0f;uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
                //test
               // vehicle_status_s vehicle_status{};
               // vehicle_status_sub.copy(&vehicle_status);
              //  if((vehicle_status.nav_state==vehicle_status_s::NAVIGATION_STATE_ACRO)&&(vehicle_status.arming_state==vehicle_status_s::ARMING_STATE_ARMED))
             //   {
              //  Vector3f pos_desi_temp1(0.673f, 1.137f, -0.8f);

              //     if (_manual_control_setpoint_sub.update(&manual_control_setpoint)){
                      //  PX4_WARN("manual_control_setpoint.2  %f %f %f %f", double(manual_control_setpoint.x),double(manual_control_setpoint.y),double(manual_control_setpoint.z),double(manual_control_setpoint.r));
              //          if((fabsf((manual_control_setpoint.x))<0.01f)){
               //              manual_control_setpoint.x=0.0f;
              //          }
               //         if((fabsf((manual_control_setpoint.y))<0.01f)){
                    //         manual_control_setpoint.y=0.0f;
                //        }
                    //    if((fabsf((manual_control_setpoint.z-0.50f))<0.1f)){
                     //        manual_control_setpoint.z=0.50f;
                     //   }
                     //   if((fabsf((manual_control_setpoint.r))<0.1f)){
                       //      manual_control_setpoint.r=0.0f;
                       // }


                   // }
             //   }
                _manual_control_setpoint_sub.copy(&manual_control_setpoint);

                theta_d=-3.1415926f*manual_control_setpoint.x*0.11f;
                phi_d=3.1415926f*manual_control_setpoint.y*0.11f;
              //  PX4_WARN("man_y   %f  ", double(manual_control_setpoint.y));

//test

                Dcmf R_d=Eulerf(phi_d,theta_d,pos_helper.psi_d);
                //R_d=R_d.transpose() ;
        //R_e=R_d.transposed()*_R;
        //math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
                matrix::Dcmf matrix1_temp=R_d.transpose() * _R ;
               float t = matrix1_temp.trace();
                    t = sqrt(1.0f + t);
                    float q_error_r=0.5f*t;
                    Vector3f q_error;
                    //warnx("sqrt(1.0f + trace)  %f ", double(t));
               //     float q_error0 = 0.5f * t;
                    t = (0.5f)/ t;
                    q_error(0) = (matrix1_temp(2,1) - matrix1_temp(1,2)) * t;
                    q_error(1) = (matrix1_temp(0,2) - matrix1_temp(2,0)) * t;
                    q_error(2) = (matrix1_temp(1,0) - matrix1_temp(0,1)) * t;


        w_r_prev=w_r;
        w_r=(lambda_q*q_error)*(-2);
      //  matrix::Vector3f w_r_deri1=(w_r-w_r_prev)/dt;
        float Q[9]={0,-q_error(2),q_error(1),q_error(2),0,-q_error(0),-q_error(1),q_error(0),0};
        matrix::Matrix<float,3, 3> Q_v(Q);
        //float[3][3] Q;
//PX4_WARN("-q_error(2) Q_v(0,1) %f %f  ", double(-q_error(2)),double(Q_v(0,1)));
        //Q_v.identity();
        //Q_v(0,0)=0.0f;
       // Q_v(0,1)=0.0f;

                matrix::Vector3f w_iusl{vehicle_angular_velocity.xyz};
                matrix::Vector3f q_v_dot=(I*q_error_r+Q_v)*w_iusl*0.5f;
                w_r_deri=(lambda_q*q_v_dot)*(-2);
              //  warnx("w_r_deri 0   %f %f %f", double(w_r_deri1(0)),double(w_r_deri1(1)), double(w_r_deri1(2)));
             //   warnx("w_r_deri real   %f %f %f", double(w_r_deri(0)),double(w_r_deri(1)), double(w_r_deri(2)));
                //w_iusl=_R.transpose()*w_iusl;
                matrix::Vector3f s_w=w_iusl-w_r;
                matrix::Vector3f tau_temp{actuator_controls.control[actuator_controls_s::INDEX_ROLL] ,actuator_controls.control[actuator_controls_s::INDEX_PITCH],actuator_controls.control[actuator_controls_s::INDEX_YAW]};
                tau_temp=tau_temp*5.0f;
               // matrix::Vector3f u_w=(matrix::Vector3f(I_b_inve*tau_temp))-((matrix::Vector3f(I_b_inve*w_iusl))%(matrix::Vector3f(I_b*w_iusl)));
                //w_r_prev=w_r;
                // warnx("pos_helper.u_w   %f %f %f", double(u_w(0)),double(u_w(1)),double(u_w(2)));
              //  fn_att_eso(w_iusl(0),w_iusl(1),w_iusl(2),u_w(0),u_w(1),u_w(2));
                //matrix::Vector3f vector_delta_esti_w{delta_esti_wr,delta_esti_wp,delta_esti_wy};
                 matrix::Vector3f vector_delta_esti_w{0.0f,0.0f,0.0f};
        //float temp_k_q=2.0;
           tau = I_b*(w_r_deri-vector_delta_esti_w)+w_iusl%(I_b*w_iusl)-(K_w*s_w)-(q_error*k_q);
        tau(1) = math::constrain(tau(1), -5.0f, 5.0f);
        tau(0) = math::constrain(tau(0), -5.0f, 5.0f);
        tau(2) = math::constrain(tau(2), -5.0f, 5.0f);
        //warnx("phi_d   theta_d  %f %f ", double(phi_d),double(theta_d));

    // Example
	// publish some data
    orb_testx_s data{};
	data.timestamp = hrt_absolute_time();
    //data.val = accel.device_id;
    data.torque_sp[0]=tau(0);
    data.torque_sp[1]=tau(1);
    data.torque_sp[2]=tau(2);    
    data.phi_d=phi_d;
    data.theta_d=theta_d;
    data.q_erro[0]=q_error(0);
    data.q_erro[1]=q_error(1);
    data.q_erro[2]=q_error(2);
	_orb_test_pub.publish(data);
   // warnx("phi   theta psi%f %f %f", double(eulertest(0)),double(eulertest(1)),double(eulertest(2)));
   //warnx("pos_helper.torque_sp   %f %f %f", double(tau(0)),double(tau(1)),double(tau(2)));

   // uORB::Subscription pos_helper_sub{ORB_ID(pos_helper)};
   // pos_helper_s		_pos_helper{};
  //  memset(&_pos_helper, 0, sizeof(_pos_helper));
   // uORB::Subscription orb_test_sub{ORB_ID(orb_test)};
         //      orb_test_s		_orb_test{};

   // actuator_controls_s actuators{};  //     memset(&_orb_test, 0, sizeof(_orb_test));

   // float f_iusl_abs;
    //float f_norm=0.0f;
      //  f_iusl_abs= Vector3f{pos_helper.thrust_sp}.length();
       // _tau= Vector3f{tau}/5;
       // _tau= Vector3f{_pos_helper.torque_sp};
       // f_norm =-(((f_iusl_abs/(4.0f))+20.89f)/0.01805f)/ 3200.0f;
       // f_norm =f_iusl_abs/20.0f;
       // f_norm = math::constrain(f_norm, 0.0f, 1.0f);
        //_tau=(((_tau/(4.0f))-1.96f)/-0.001664f)/3200.0f;
       // _tau=tau/5.0f;

       // _tau(0) = math::constrain(_tau(0), -0.0005f, 0.0005f);
     //   _tau(1) = math::constrain(_tau(1), -0.0005f, 0.0005f);
     //   _tau(2) = math::constrain(_tau(2), -0.0005f, 0.0005f);
      //  actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(_tau(0)) ? (_tau(0)) : 0.0f;
      //  actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE((_tau(1))) ? (_tau(1)) : 0.0f;
      //  actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(_tau(2)) ? (_tau(2)): 0.0f;
        //actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
       // actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
      //  actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
     //   actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(f_norm) ? (f_norm) : 0.0f;
      //  actuators.timestamp = hrt_absolute_time();
      //  _actuators_0_pub.publish(actuators);

	perf_end(_loop_perf);
}

int WorkItemExample::task_spawn(int argc, char *argv[])
{
	WorkItemExample *instance = new WorkItemExample();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int WorkItemExample::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemExample::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemExample::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int work_item_example_main(int argc, char *argv[])
{
	return WorkItemExample::main(argc, argv);
}
