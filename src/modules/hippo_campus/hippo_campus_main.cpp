
/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file hippo_campus_main.cpp
 *
 * HippoCampus Underwater Attitude Controller. Feed Forward RC Input except roll. Balance roll.
 *
 * Based on rover steering control example by Lorenz Meier <lorenz@px4.io>
 *
 * @author Max Kirchhoff
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <geo/geo.h>


/* Prototypes */

/**
 * HippoCampus app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int hippo_campus_main(int argc, char *argv[]);

class HippoCampusTest {
public:
    /**
     * Constructor
     */
    HippoCampusTest();

    /**
     * Destructor, also kills the main task
     */
    ~HippoCampusTest();

    /**
     * Start the underwater attitude control task.
     *
     * @return		OK on success.
     */
    int		start();

private:

    bool	_task_should_exit;		/**< if true, task_main() should exit */
    int		_control_task;			/**< task handle */


    orb_advert_t     _v_att_sp_pub;          /**< vehicle attitude setpoint publication */
    orb_advert_t     _v_rates_sp_pub;          /**< vehicle rates setpoint publication */
    int              _v_att_sub;             /**< vehicle attitude subscription */

    struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
    struct vehicle_rates_setpoint_s     _v_rates_sp;        /**< vehicle rates setpoint */
    struct vehicle_attitude_s           _v_att;             /**< vehicle attitude */


    void send_angles (int yaw_deg, int pitch_deg, int roll_deg);
    void send_angles_rollignored (int pitch_deg, int yaw_deg);
    void send_rates(float roll_rate, float pitch_rate, float yaw_rate, float thrust);
    void thrust_brake(float thrust_power, float break_power, int thrust_time_msec, int brake_time_msec);
    void angle_setpoints();

    /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);

    /**
     * Main attitude control task.
     */
    void		task_main();
};

namespace hippo_campus
{

HippoCampusTest	*g_control;
}



HippoCampusTest::HippoCampusTest() :

    _task_should_exit(false),
    _control_task(-1),
    _v_att_sp_pub(nullptr),
    _v_rates_sp_pub(nullptr),
    _v_att_sub(-1)

{
    memset(&_v_att_sp, 0, sizeof(_v_att_sp));
    memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
    memset(&_v_att, 0, sizeof(_v_att));
}

HippoCampusTest::~HippoCampusTest()
{
    if (_control_task != -1) {
        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                px4_task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }


    hippo_campus::g_control = nullptr;
}


 void HippoCampusTest::send_angles(int yaw_deg, int pitch_deg, int roll_deg){

     PX4_WARN("roll%8.4f\tpitch%8.4f\tyaw%8.4f",(double)roll_deg,(double)pitch_deg,(double)yaw_deg);

    //convert to rad
    float r=float(roll_deg)/180.0f*M_PI_F;
    float p=float(pitch_deg)/180.0f*M_PI_F;
    float y=float(yaw_deg)/180.0f*M_PI_F;

    //set angle setpoints
    _v_att_sp.roll_body=r;
    _v_att_sp.pitch_body=p;
    _v_att_sp.yaw_body=y;


    //----------- ZYX Rotation-Matrix -----------//
    math::Matrix<3,3> R_from_euler;
    R_from_euler.from_euler(r,p,y);

    // set R_body setpoint
    memcpy(&_v_att_sp.R_body[0], &R_from_euler.data[0], sizeof(_v_att_sp.R_body));

    //--------------- Quaternion ----------------//
    math::Quaternion q_d;
    q_d.from_dcm(R_from_euler);

    // normalize if necessary
    for(int i=0;i<3;i++){
        if (fabs((double)(q_d.normalized()(i)-q_d(i)))>0.0001)
            q_d.normalize();
        }
    // set q_d setpoint
    memcpy(&_v_att_sp.q_d[0], &q_d.data[0], sizeof(_v_att_sp.q_d));

    //------------publish setpoints --------------//
    orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);


    //--------- if desired, wait until setpoint is reached---------//
    bool roll_ok,pitch_ok,yaw_ok;
    int wait;
    param_get(param_find("HC_WAIT"), &(wait));
    while(wait){
        usleep(2000);
        orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

        // roll +-5 degrees, ignore when setpoint is facing up/down
        roll_ok=(abs((int)((_v_att.roll-r)*100))<9||p>1.5f||p<-1.5f);

        // pitch +-5degrees
        pitch_ok=(abs((int)((_v_att.pitch-p)*100))<9);

        // yaw +-5degrees
        yaw_ok=(abs((int)((_v_att.yaw-y)*100))<9);

        if(roll_ok && pitch_ok && yaw_ok) return;
    }

    param_get(param_find("HC_M_TIME"), &(wait));
    sleep(wait);
}

 void HippoCampusTest::send_angles_rollignored(int pitch_deg, int yaw_deg){

        //convert to rad
        float pitch=float(pitch_deg)/180.0f*3.14159265f;
        float yaw=float(yaw_deg)/180.0f*3.14159265f;

        _v_att_sp.roll_body=5.0f;
        _v_att_sp.pitch_body=pitch;
        _v_att_sp.yaw_body=yaw;

        orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);


        // wait until setpoint is reached
        bool pitch_ok,yaw_ok;
        float p;
        float y;
        float cr;
        float sr;

        while(true){
        usleep(2000);
        orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
        p=_v_att.pitch;
        y=_v_att.yaw;
        cr=cosf(_v_att.roll);
        sr=sinf(_v_att.roll);

        // pitch +-5degrees
        pitch_ok=(p*cr+y*sr-pitch*cr-yaw*sr)<0.05f;
        yaw_ok=(y*cr+p*sr-yaw*cr-pitch*sr)<0.05f;

        if(pitch_ok && yaw_ok) break;
        }
    }

 void HippoCampusTest::send_rates(float roll_rate, float pitch_rate, float yaw_rate, float thrust = 0.0f){
     float thrust_scale;
     param_get(param_find("HC_T_SCALE"), &(thrust_scale));
     PX4_WARN("rollrate %8.4f\tpitchrate %8.4f\tyawrate %8.4f\tthrust %8.4f",(double)roll_rate,(double)pitch_rate,(double)yaw_rate,(double)thrust);
    _v_rates_sp.roll = roll_rate;
    _v_rates_sp.pitch = pitch_rate;
    _v_rates_sp.yaw = yaw_rate;
    _v_rates_sp.thrust = thrust*thrust_scale;

    orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);
}

 void HippoCampusTest::thrust_brake(float thrust_power, float break_power, int thrust_time_msec, int brake_time_msec){
     float thrust_scale;
     param_get(param_find("HC_T_SCALE"), &(thrust_scale));


     _v_att_sp.thrust=thrust_power*thrust_scale;
    orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);

    usleep(thrust_time_msec*1000);

    _v_att_sp.thrust=break_power*thrust_scale;
    orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);

    usleep(brake_time_msec*1000);

    _v_att_sp.thrust=0.0f;
    orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);

}

 void HippoCampusTest::angle_setpoints(){

      /** Roll, Pitch and Yaw setpoints in degrees (ZYX convention)
       *  Yaw:    -180...180
       *  Pitch:   -90...90
       *  Roll:    -90...90
       **/
     int hc_mode;
     int control_mode; //0: manual control || 1: assisted manual || 2:  standard control || 3: direct mapping of rates_sp
     param_get(param_find("HC_MODE"), &(hc_mode));

     int hc_time;
     param_get(param_find("HC_TIME"), &(hc_time));

    //------------------------------------------------------------------------
        if(hc_mode == 0){
            control_mode = 2;
            param_set(param_find("UW_CONTROL_MODE"), &control_mode);
              warnx("setpoint 1");
              send_angles(0,0,0);


              warnx("next setpoint");
              send_angles(90,0,0);


              warnx("next setpoint");
              send_angles(180,0,0);


              warnx("next setpoint");
              send_angles(-90,0,0);


              return;
            }
    //------------------------------------------------------------------------
        if(hc_mode == 1){
            control_mode = 2;
            param_set(param_find("UW_CONTROL_MODE"), &control_mode);
              warnx("box");
              send_angles(0,0,0);


              thrust_brake(0.1f,-0.2f,500,180);
              sleep(1);
              send_angles(90,0,0);


              thrust_brake(0.1f,-0.2f,500,180);
              sleep(1);
              send_angles(180,0,0);
              thrust_brake(0.1f,-0.2f,500,180);
              sleep(1);
              send_angles(-90,0,0);
              thrust_brake(0.1f,-0.2f,500,180);
              sleep(1);
              send_angles(0,0,0);
        return;
        }
    //------------------------------------------------------------------------
      if(hc_mode == 2){
          control_mode = 2;
          param_set(param_find("UW_CONTROL_MODE"), &control_mode);
          warnx("helix");
          send_angles(0,20,0);
          send_rates(0.0f,0.0f,0.5f);
          thrust_brake(0.07f,0.03f,10000,200);
          send_rates(0.0f,0.0f,0.0f);
          sleep(2);

          warnx("next setpoint");
          send_angles(0,0,0);
          warnx("end");
         return;
      }

    //-------------------spin-----------------------------------------------------
      if (hc_mode == 3){

          control_mode = 2;
          param_set(param_find("UW_CONTROL_MODE"), &control_mode);

          send_angles(0,20,0);

        warnx("spin");
          control_mode = 4;
          param_set(param_find("UW_CONTROL_MODE"), &control_mode);

          send_rates(0.6f,0.0f,0.07f,0.005f);

          sleep(10);
          control_mode = 2;
          param_set(param_find("UW_CONTROL_MODE"), &control_mode);
          send_rates(0.0f,0.0f,0.0f,0.0f);
          orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
          send_angles(int(_v_att.yaw/M_PI_F*180.0f),0,0);


           send_angles(0,0,0);


        return;
      }
    //-------------------barrel-----------------------------------------------------
      if (hc_mode == 4){
          control_mode = 3;
          param_set(param_find("UW_CONTROL_MODE"), &control_mode);
          send_rates(0.6f,0.2f,0.0f,0.01f);
          sleep(5);

        return;
      }

    //--------------------external setpoint----------------------------------------------------
      if (hc_mode == 5){
          int ext_yaw,ext_pitch,ext_roll;
          param_get(param_find("HC_YAW"), &(ext_yaw));
          param_get(param_find("HC_PITCH"), &(ext_pitch));
          param_get(param_find("HC_ROLL"), &(ext_roll));
          control_mode = 2;
          param_set(param_find("UW_CONTROL_MODE"), &control_mode);
          send_angles(ext_yaw,ext_pitch,ext_roll);

        return;
      }


      //VORLAGE -----------------------------------------------------------------------
      if (hc_mode == 3){
          control_mode = 2;
          param_set(param_find("UW_CONTROL_MODE"), &control_mode);


          control_mode = 2;
          param_set(param_find("UW_CONTROL_MODE"), &control_mode);

        return;
      }


}

 void HippoCampusTest::task_main_trampoline(int argc, char *argv[])
{
    hippo_campus::g_control->task_main();
}

 void HippoCampusTest::task_main(){


    _v_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);
     _v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
    _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
    _v_att_sp.roll_body=_v_att.roll;
    _v_att_sp.pitch_body=_v_att.pitch;
    _v_att_sp.yaw_body=_v_att.yaw;
    orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);

    //countdown
    warnx("3");
    usleep(1000000);
    warnx("2");
    usleep(1000000);
    warnx("1");
    usleep(1000000);


    angle_setpoints();

    warnx("end");

    _v_att_sp.thrust=0.0f;
    orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);


    _task_should_exit=true;
    _control_task = -1;
    delete hippo_campus::g_control;
    hippo_campus::g_control = nullptr;
    return;
}



int HippoCampusTest::start()
{
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("hippo_campus",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       1500,
                       (px4_main_t)&HippoCampusTest::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}


 int hippo_campus_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("usage: hippo_campus {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (hippo_campus::g_control != nullptr) {
            warnx("already running");
            return 1;
        }

        hippo_campus::g_control = new HippoCampusTest;

        if (hippo_campus::g_control == nullptr) {
            warnx("alloc failed");
            return 1;
        }

        if (OK != hippo_campus::g_control->start()) {
            delete hippo_campus::g_control;
            hippo_campus::g_control = nullptr;
            warnx("start failed");
            return 1;
        }
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (hippo_campus::g_control == nullptr) {
            warnx("not running");
            return 1;
        }

        delete hippo_campus::g_control;
        hippo_campus::g_control = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (hippo_campus::g_control) {
            warnx("running");
            return 0;

        } else {
            warnx("not running");
            return 1;
        }
    }

    warnx("unrecognized command");
    return 1;
}
