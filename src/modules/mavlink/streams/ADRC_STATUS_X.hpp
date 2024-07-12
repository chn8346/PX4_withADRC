#ifndef ADRC_STATUS_X_STREAM_HPP
#define ADRC_STATUS_X_STREAM_HPP

#include <uORB/topics/ladrc_status.h>
//#include <mavlink.h>
//#include <mavlink_stream.h>

class MavlinkStreamAdrcStatusX: public MavlinkStream
{
public:
    const char *get_name() const { return MavlinkStreamAdrcStatusX::get_name_static(); }

    static const char *get_name_static() { return "ADRC_STATUS_X"; }

    static uint16_t get_id_static(){ return MAVLINK_MSG_ID_ADRC_STATUS_X; }

    uint16_t get_id(){ return get_id_static(); }

    static MavlinkStream *new_instance(Mavlink *mavlink){ return new MavlinkStreamAdrcStatusX(mavlink); }

    unsigned get_size(){ return MAVLINK_MSG_ID_ADRC_STATUS_X_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

private:
    uORB::Subscription _sub_x{ORB_ID(ladrc_status_ratex)};

    /* do not allow top copying this class */
    MavlinkStreamAdrcStatusX(MavlinkStreamAdrcStatusX &);
    MavlinkStreamAdrcStatusX& operator = (const MavlinkStreamAdrcStatusX &);

protected:
    explicit MavlinkStreamAdrcStatusX(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

    bool send() override
    {
        ladrc_status_s ladrc_status_uorb_x;    // uORB topic rate x

	if (_sub_x.update(&ladrc_status_uorb_x)) {
	    __mavlink_adrc_status_x_t ladrc_status_mavlink;  //make sure mavlink is the definition of your custom MAVLink message

	    ladrc_status_mavlink.v = ladrc_status_uorb_x.v;
	    ladrc_status_mavlink.v1 = ladrc_status_uorb_x.v1;
	    ladrc_status_mavlink.v2 = ladrc_status_uorb_x.v2;

	    ladrc_status_mavlink.e1 = ladrc_status_uorb_x.e1;
	    ladrc_status_mavlink.e2 = ladrc_status_uorb_x.e2;

	    ladrc_status_mavlink.u0 = ladrc_status_uorb_x.u0;
	    ladrc_status_mavlink.u = ladrc_status_uorb_x.u;

	    ladrc_status_mavlink.y = ladrc_status_uorb_x.y;

	    ladrc_status_mavlink.z1 = ladrc_status_uorb_x.z1;
	    ladrc_status_mavlink.z2 = ladrc_status_uorb_x.z2;
	    ladrc_status_mavlink.z3 = ladrc_status_uorb_x.z3;

	    mavlink_msg_adrc_status_x_send_struct(_mavlink->get_channel(), &ladrc_status_mavlink);

	    return true;
	}

        return true;
    }
};

#endif // ADRC_STATUS_X_STREAM_HPP
