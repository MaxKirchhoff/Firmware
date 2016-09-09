
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

    struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */


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
    _v_att_sp_pub(nullptr)

{
    memset(&_v_att_sp, 0, sizeof(_v_att_sp));
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









void HippoCampusTest::task_main_trampoline(int argc, char *argv[])
{
    hippo_campus::g_control->task_main();
}


void HippoCampusTest::task_main()
{


_v_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);

//set setpoint quaternion
math::Quaternion q_d;
_v_att_sp.thrust=1.0f;
orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);
    //countdown
      warnx("5");
      usleep(1000000);
      _v_att_sp.thrust=0.0f;
      orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);
      warnx("4");
      usleep(1000000);
      warnx("3");
      usleep(1000000);
      warnx("2");
      usleep(1000000);
      warnx("1");
      usleep(1000000);


      warnx("setpoint 1");
      q_d.from_euler(1.0f,0.0f,0.0f);
      memcpy(&_v_att_sp.q_d[0], &q_d.data[0], sizeof(_v_att_sp.q_d));
      orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);
      usleep(5000000);

      warnx("setpoint 2");
      q_d.from_euler(0.0f,0.0f,0.0f);
      memcpy(&_v_att_sp.q_d[0], &q_d.data[0], sizeof(_v_att_sp.q_d));
      orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);
      usleep(5000000);

      warnx("setpoint 3");
      q_d.from_euler(0.0f,3.1415f,0.0f);
      memcpy(&_v_att_sp.q_d[0], &q_d.data[0], sizeof(_v_att_sp.q_d));
      orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);
      usleep(5000000);

    warnx("tschüss");
     _task_should_exit=true;




    _control_task = -1;
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
