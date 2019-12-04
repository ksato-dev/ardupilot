#include "Copter.h"
#include <cstdint>

#if MODE_NEW_ENABLED == ENABLED

#if 1
const float xyz_table[][3] = {
    {0.0, 0.0, 5},
    {5.0, 0.0, 5.5},
    {10.0, 0.0, 6.0},
    {15.0, 0.0, 6.25},
    {15.0, 5.0, 6.5},
    {15.0, 10.0, 7.0},
    {15.0, 15.0, 7.5}};

#else 
const float xyz_table[][3] = {  // [m]
    {1, 0, 5},
    {0.995004165278026, 0.099833416646828, 5.1},
    {0.980066577841242, 0.198669330795061, 5.2},
    {0.955336489125606, 0.29552020666134, 5.3},
    {0.921060994002885, 0.38941834230865, 5.4},
    {0.877582561890373, 0.479425538604203, 5.5},
    {0.825335614909678, 0.564642473395035, 5.6},
    {0.764842187284488, 0.644217687237691, 5.7},
    {0.696706709347165, 0.717356090899523, 5.8},
    {0.621609968270664, 0.783326909627483, 5.9},
    {0.54030230586814, 0.841470984807896, 6},
    {0.453596121425577, 0.891207360061435, 6.1},
    {0.362357754476673, 0.932039085967226, 6.2},
    {0.267498828624587, 0.963558185417193, 6.3},
    {0.169967142900241, 0.98544972998846, 6.4},
    {0.070737201667703, 0.997494986604055, 6.5},
    {-0.029199522301289, 0.999573603041505, 6.6},
    {-0.128844494295525, 0.991664810452468, 6.7},
    {-0.227202094693087, 0.973847630878195, 6.8},
    {-0.323289566863504, 0.946300087687414, 6.9},
    {-0.416146836547142, 0.909297426825682, 7},
    {-0.504846104599858, 0.863209366648874, 7.1},
    {-0.588501117255346, 0.80849640381959, 7.2},
    {-0.666276021279824, 0.74570521217672, 7.3},
    {-0.737393715541246, 0.675463180551151, 7.4},
    {-0.801143615546934, 0.598472144103957, 7.5},
    {-0.856888753368947, 0.515501371821464, 7.6},
    {-0.904072142017061, 0.42737988023383, 7.7},
    {-0.942222340668658, 0.334988150155905, 7.8},
    {-0.970958165149591, 0.239249329213982, 7.9},
    {-0.989992496600445, 0.141120008059867, 8},
    {-0.999135150273279, 0.041580662433291, 8.1},
    {-0.998294775794753, -0.05837414342758, 8.2},
    {-0.987479769908865, -0.157745694143249, 8.3},
    {-0.966798192579461, -0.255541102026832, 8.4},
    {-0.936456687290796, -0.35078322768962, 8.5},
    {-0.896758416334147, -0.442520443294852, 8.6},
    {-0.848100031710408, -0.529836140908493, 8.7},
    {-0.790967711914417, -0.611857890942719, 8.8},
    {-0.72593230420014, -0.687766159183974, 8.9},
    {-0.653643620863612, -0.756802495307928, 9},
    {-0.574823946533269, -0.818277111064411, 9.1},
    {-0.490260821340699, -0.871575772413588, 9.2},
    {-0.400799172079975, -0.916165936749455, 9.3},
    {-0.307332869978419, -0.951602073889516, 9.4},
    {-0.21079579943078, -0.977530117665097, 9.5},
    {-0.112152526935054, -0.993691003633465, 9.6},
    {-0.012388663462891, -0.999923257564101, 9.7},
    {0.087498983439447, -0.996164608835841, 9.8},
    {0.186512369422576, -0.982452612624332, 9.9},
    {0.283662185463226, -0.958924274663139, 10},
    {0.377977742712981, -0.925814682327732, 10.1},
    {0.468516671300377, -0.883454655720153, 10.2},
    {0.554374336179162, -0.832267442223901, 10.3},
    {0.634692875942635, -0.772764487555987, 10.4},
    {0.70866977429126, -0.705540325570392, 10.5},
    {0.77556587851025, -0.631266637872321, 10.6},
    {0.83471278483916, -0.550685542597638, 10.7},
    {0.885519516941319, -0.464602179413757, 10.8},
    {0.927478430744036, -0.373876664830236, 10.9},
    {0.960170286650366, -0.279415498198926, 11},
    {0.983268438442585, -0.182162504272095, 11.1},
    {0.996542097023217, -0.083089402817496, 11.2},
    {0.999858636383415, 0.016813900484351, 11.3},
    {0.993184918758193, 0.116549204850494, 11.4},
    {0.976587625728024, 0.215119988087815, 11.5},
    {0.950232591958529, 0.311541363513379, 11.6},
    {0.914383148235319, 0.404849920616598, 11.7},
    {0.869397490349825, 0.494113351138609, 11.8},
    {0.815725100125357, 0.5784397643882, 11.9},
    {0.753902254343305, 0.656986598718789, 12},
    {0.684546666442806, 0.728969040125876, 12.1},
    {0.608351314532255, 0.793667863849153, 12.2},
    {0.526077517381105, 0.850436620628565, 12.3},
    {0.43854732757439, 0.898708095811627, 12.4},
    {0.346635317835026, 0.937999976774739, 12.5},
    {0.251259842582255, 0.967919672031487, 12.6},
    {0.153373862037864, 0.988168233877, 12.7},
    {0.053955420562649, 0.998543345374605, 12.8},
    {-0.046002125639537, 0.998941341839772, 12.9},
    {-0.145500033808614, 0.989358246623382, 13},
    {-0.243544153735791, 0.969889810845086, 13.1},
    {-0.339154860983836, 0.940730556679772, 13.2},
    {-0.431376844970621, 0.902171833756293, 13.3},
    {-0.519288654116686, 0.85459890808828, 13.4},
    {-0.602011902684824, 0.79848711262349, 13.5},
    {-0.678720047320012, 0.734397097874113, 13.6},
    {-0.7486466455974, 0.662969230082182, 13.7},
    {-0.811093014061656, 0.584917192891762, 13.8},
    {-0.865435209241112, 0.501020856457885, 13.9},
    {-0.911130261884677, 0.412118485241757, 14},
    {-0.947721602131112, 0.319098362349352, 14.1},
    {-0.974843621404164, 0.222889914100246, 14.2},
    {-0.992225325452603, 0.124454423507062, 14.3},
    {-0.999693042035207, 0.024775425453358, 14.4},
    {-0.997172156196378, -0.075151120461809, 14.5},
    {-0.984687855794127, -0.174326781222981, 14.6},
    {-0.96236487983131, -0.271760626410944, 14.7},
    {-0.930426272104753, -0.366479129251928, 14.8},
    {-0.889191152625361, -0.457535893775321, 14.9},
    {-0.839071529076452, -0.54402111088937, 15},
    {-0.780568180169183, -0.625070648892883, 15.1},
    {-0.714265652027199, -0.699874687593544, 15.2},
    {-0.640826417594993, -0.767685809763582, 15.3},
    {-0.560984257427229, -0.827826469085654, 15.4},
    {-0.475536927995992, -0.87969575997167, 15.5},
    {-0.385338190771828, -0.922775421612807, 15.6},
    {-0.291289281721344, -0.956635016270188, 15.7},
    {-0.194329906455335, -0.980936230066492, 15.8},
    {-0.095428851000951, -0.995436253306377, 15.9},
    {0.004425697988051, -0.999990206550704, 16},
    {0.104236026865699, -0.994552588203989, 16.1},
    {0.203004863818752, -0.979177729151317, 16.2},
    {0.299745343277015, -0.954019249902089, 16.3},
    {0.393490866347891, -0.919328525664676, 16.4},
    {0.483304758753006, -0.875452174688428, 16.5},
    {0.568289629767975, -0.822828594968708, 16.6},
    {0.647596338653877, -0.761983583919032, 16.7},
    {0.720432478990839, -0.693525084777122, 16.8},
    {0.786070296141039, -0.618137112237033, 16.9},
    {0.843853958732492, -0.536572918000435, 17},
    {0.893206111509323, -0.4496474645346, 17.1},
    {0.933633644074638, -0.358229282236827, 17.2},
    {0.96473261788661, -0.263231791365801, 17.3},
    {0.986192302278864, -0.165604175448309, 17.4},
    {0.997798279178581, -0.066321897351201, 17.5},
    {0.999434585501005, 0.033623047221139, 17.6},
    {0.991084871814253, 0.133232041419944, 17.7},
    {0.972832565697435, 0.231509825101539, 17.8},
    {0.944860038159861, 0.327474439137693, 17.9},
    {0.907446781450196, 0.420167036826641, 18},
    {0.860966616462306, 0.508661464372375, 18.1},
    {0.80588395764045, 0.592073514707225, 18.2},
    {0.74274917270367, 0.669569762196602, 18.3},
    {0.672193083553468, 0.740375889952449, 18.4},
    {0.594920663309892, 0.803784426551621, 18.5},
    {0.511703992453147, 0.859161814856497, 18.6},
    {0.423374544450664, 0.905954742308463, 18.7},
    {0.330814877949047, 0.943695669444105, 18.8},
    {0.234949818539823, 0.972007501394976, 18.9},
    {0.136737218207834, 0.99060735569487, 19},
    {0.037158384790825, 0.999309388747918, 19.1},
    {-0.062791722924084, 0.998026652716362, 19.2},
    {-0.162114436499718, 0.986771964274613, 19.3},
    {-0.259817356213756, 0.965657776549277, 19.4},
    {-0.354924266788705, 0.934895055524683, 19.5},
    {-0.446484891412267, 0.894791172140503, 19.6},
    {-0.533584386589119, 0.845746831142933, 19.7},
    {-0.615352482954721, 0.788252067375316, 19.8},
    {-0.690972180719126, 0.722881349511976, 19.9},
    {-0.759687912858821, 0.650287840157117, 20},
    {-0.820813094492669, 0.571196869659987, 20.1},
    {-0.873736983011081, 0.486398688853798, 20.2},
    {-0.917930780414293, 0.396740573130612, 20.3},
    {-0.95295291688718, 0.303118356745702, 20.4},
    {-0.978453462818884, 0.206467481937797, 20.5},
    {-0.994177625183815, 0.107753652299442, 20.6},
    {-0.99996829334934, 0.007963183785936, 20.7},
    {-0.995767608873289, -0.091906850227682, 20.8},
    {-0.981617543606384, -0.190858581374189, 20.9},
    {-0.957659480323385, -0.287903316665065, 21},
    {-0.92413280007313, -0.382071417184009, 21.1},
    {-0.881372490362235, -0.472421986398466, 21.2},
    {-0.829805798070649, -0.558052271286779, 21.3},
    {-0.769947960542069, -0.63810668234795, 21.4},
    {-0.702397057502713, -0.711785342369123, 21.5},
    {-0.627828035246386, -0.778352078534298, 21.6},
    {-0.546985962794236, -0.837141778019747, 21.7},
    {-0.460678587411363, -0.887567033581504, 21.8},
    {-0.36976826386317, -0.92912401273437, 21.9},
    {-0.275163338051597, -0.961397491879557, 22},
    {-0.177809071123116, -0.984065005081643, 22.1},
    {-0.07867819473184, -0.996900066041596, 22.2},
    {0.021238808173646, -0.999774431073011, 22.3},
    {0.120943599928478, -0.992659380470633, 22.4},
    {0.219439963211459, -0.975626005468158, 22.5},
    {0.315743754919243, -0.948844497918124, 22.6},
    {0.40889273939888, -0.912582449791185, 22.7},
    {0.497956202788415, -0.867202179485581, 22.8},
    {0.582044252402125, -0.813157111661486, 22.9},
    {0.66031670824408, -0.750987246771676, 23},
    {0.731991497808947, -0.6813137655555, 23.1},
    {0.796352470291923, -0.604832822406284, 23.2},
    {0.852756552130873, -0.522308589626732, 23.3},
    {0.90064017238477, -0.434565622071893, 23.4},
    {0.939524893748256, -0.342480618469613, 23.5},
    {0.96902219293905, -0.246973661736621, 23.6},
    {0.988837342694146, -0.148999025814199, 23.7},
    {0.99877235658721, -0.049535640878368, 23.8},
    {0.998727967243501, 0.050422687806815, 23.9},
    {0.988704618186669, 0.149877209662952, 24},
    {0.96880245940721, 0.24783420798296, 24.1},
    {0.93922034669687, 0.343314928819899, 24.2},
    {0.900253854747304, 0.435365360372893, 24.3},
    {0.852292323865462, 0.523065765157699, 24.4},
    {0.795814969813944, 0.605539869719601, 24.5},
    {0.731386095645497, 0.681963620068136, 24.6},
    {0.659649453373459, 0.751573415352151, 24.7},
    {0.581321811814436, 0.813673737507105, 24.8},
    {0.497185794871202, 0.867644100641669, 24.9},
    {0.408082061813392, 0.912945250727628, 25}};
#endif

static uint32_t count_val = 1;

bool ModeNew::init(bool ignore_checks)
{
    hal.console->printf("hogehoge\n");
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return false;
    }

    pos_control->init_xy_controller();
    // pos_control->set_target_to_stopping_point_xy();
    // pos_control->set_target_to_stopping_point_z();
    pos_control->set_desired_accel_xy(0.0f,0.0f);
    pos_control->set_desired_velocity_xy(0.0f,0.0f);

    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(1000.0);
    pos_control->set_max_accel_xy(100.0);
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // get stopping point
    // const Vector3f& stopping_point = pos_control->get_pos_target();
    const Vector3f& stopping_point = inertial_nav.get_position();

    _center.x = stopping_point.x;
    _center.y = stopping_point.y;
    _center.z = stopping_point.z;

    hal.console->printf("x:%f, y:%f, z:%f\n",_center.x, _center.y, _center.z);

    // initialise circle controller including setting the circle center based on vehicle speed
    // copter.circle_nav->init();
    this->_xyz_counter = 0;

    this->_pos_target_cm.x = xyz_table[0][0] * 100.0 + _center.x;
    this->_pos_target_cm.y = xyz_table[0][1] * 100.0 + _center.y;
    this->_pos_target_cm.z = xyz_table[0][2] * 100.0 + _center.z;
    this->_pre_pos_target_cm = _pos_target_cm;

    return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void ModeNew::run()
{
    static float distance_cm = 0.0;

    if (distance_cm >= 20.0) {
        pos_control->set_alt_target(this->_pre_pos_target_cm.z);
        pos_control->update_z_controller();
        pos_control->update_xy_controller();

        float next_yaw = get_bearing_cd(inertial_nav.get_position(), this->_pos_target_cm);
        attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(),
                                                           pos_control->get_pitch(),
                                                           next_yaw, true);
        // hal.console->printf("distance to target:%f\n", distance_cm);
        distance_cm = pos_control->get_distance_to_target();
        return;
    }

    // initialize speeds and accelerations
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set auto_yaw
//    static float curr_yaw = 0.0;
//    curr_yaw += 3.0;
//    auto_yaw.set_fixed_yaw(curr_yaw, 0.0, 0, false);

    this->_pos_target_cm.x = xyz_table[this->_xyz_counter][0] * 100.0 + _center.x;
    this->_pos_target_cm.y = xyz_table[this->_xyz_counter][1] * 100.0 + _center.y;
    this->_pos_target_cm.z = 10.0 * xyz_table[this->_xyz_counter][2] * 100.0;

    // control xy
    pos_control->set_xy_target(this->_pos_target_cm.x, this->_pos_target_cm.y);
    pos_control->set_desired_velocity_xy(20.0, 20.0);
    float next_yaw = get_bearing_cd(inertial_nav.get_position(), this->_pos_target_cm);
    pos_control->update_xy_controller();
    distance_cm = pos_control->get_distance_to_target();

    this->_pre_pos_target_cm = this->_pos_target_cm;

    // control z
    // pos_control->set_alt_target_to_current_alt();
    pos_control->set_alt_target(this->_pos_target_cm.z);
    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(),
                                                       pos_control->get_pitch(),
                                                       next_yaw, true);

    // pos_control->set_alt_target_to_current_alt();
    pos_control->set_alt_target(this->_pos_target_cm.z);
    pos_control->update_z_controller();

    // attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);

    this->update_xyz_counter();
}

uint32_t ModeNew::wp_distance() const
{
    return copter.circle_nav->get_distance_to_target();
}

int32_t ModeNew::wp_bearing() const
{
    return copter.circle_nav->get_bearing_to_target();
}

void ModeNew::update_xyz_counter() {

    // hal.console->printf("count:%d\n", this->_xyz_counter);

    uint32_t num_xyz = (sizeof(xyz_table) / sizeof(xyz_table[0][0])) / 3;
    if (this->_xyz_counter >= num_xyz - 1)  // warning: don't refer to out of array's range.
        count_val = -1;
    else if (this->_xyz_counter == 0) {
        count_val = 1;
    }

    this->_xyz_counter += count_val;
    return;
};
#endif
