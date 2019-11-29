#include "Copter.h"

#if MODE_NEW_ENABLED == ENABLED

bool ModeNew::init(bool ignore_checks)
{
    pilot_yaw_override = false;

    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    copter.circle_nav->init();

    return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void ModeNew::run()
{
    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set auto_yaw
    static float curr_yaw = 0.0;
    curr_yaw += 3.0;
    auto_yaw.set_fixed_yaw(curr_yaw, 0.0, 0, false);

    // control xy
    _pos_target_cm.x += 1.0;
    _pos_target_cm.y += 1.0;
    pos_control->set_xy_target(_pos_target_cm.x, _pos_target_cm.y);
    pos_control->update_xy_controller();

    // control z
    // pos_control->set_alt_target_to_current_alt();
    pos_control->set_alt_target(300.0);
    pos_control->update_z_controller();

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(),
                                                       pos_control->get_pitch(),
                                                       auto_yaw.yaw(), true);

    // pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
}

uint32_t ModeNew::wp_distance() const
{
    return copter.circle_nav->get_distance_to_target();
}

int32_t ModeNew::wp_bearing() const
{
    return copter.circle_nav->get_bearing_to_target();
}

#endif
