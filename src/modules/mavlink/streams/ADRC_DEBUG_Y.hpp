#ifndef ADRC_DEBUG_Y_STREAM_HPP
#define ADRC_DEBUG_Y_STREAM_HPP

// #include <uORB/topics/ladrc_status.h>
#include <uORB/topics/ladrc_control_dis.h>
//#include <mavlink.h>
//#include <mavlink_stream.h>

class MavlinkStreamAdrcDebugAY: public MavlinkStream
{
public:
    const char *get_name() const { return MavlinkStreamAdrcDebugAY::get_name_static(); }

    static const char *get_name_static() { return "ADRC_DEBUG_AY_DIS"; }

    static uint16_t get_id_static(){ return MAVLINK_MSG_ID_ADRC_DEBUG_AY_DIS; }

    uint16_t get_id(){ return get_id_static(); }

    static MavlinkStream *new_instance(Mavlink *mavlink){ return new MavlinkStreamAdrcDebugAY(mavlink); }

    unsigned get_size(){ return MAVLINK_MSG_ID_ADRC_DEBUG_AY_DIS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

private:
    uORB::Subscription _ladrc_control_dis_sub{ORB_ID(ladrc_angley_dis)};

    /* do not allow top copying this class */
    MavlinkStreamAdrcDebugAY(MavlinkStreamAdrcDebugAY &);
    MavlinkStreamAdrcDebugAY& operator = (const MavlinkStreamAdrcDebugAY &);

protected:
    explicit MavlinkStreamAdrcDebugAY(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

    bool send() override
    {
        ladrc_control_dis_s ladrc_debug_ay;    // uORB topic data

	if (_ladrc_control_dis_sub.update(&ladrc_debug_ay)) {
	    __mavlink_adrc_debug_ay_dis_t ladrc_debug_mavlink;  //make sure mavlink is the definition of your custom MAVLink message

	    ladrc_debug_mavlink.pid = ladrc_debug_ay.pid_val;
	    ladrc_debug_mavlink.adrc = ladrc_debug_ay.adrc_val;
	    ladrc_debug_mavlink.dis = ladrc_debug_ay.dis_of_val;

	    mavlink_msg_adrc_debug_ay_dis_send_struct(_mavlink->get_channel(), &ladrc_debug_mavlink);

	    return true;
	}

        return true;
    }
};

#endif // ADRC_DEBUG_Y_STREAM_HPP
