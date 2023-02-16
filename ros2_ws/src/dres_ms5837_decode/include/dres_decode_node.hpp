#ifndef DRES_DECODE_NODE_MS5837_H
#define DRES_DECODE_NODE_MS5837_H

#include <linux/can.h>
#include <net/if.h>

#include "scion_types/msg/can_frame.hpp"
#include "ms5837_handler.hpp"

class DresDecodeNode : public rclcpp::Node
{
	public:
		DresDecodeNode();
	private:
		rclcpp::Subscription<scion_types::msg::CanFrame>::SharedPtr _dres_mb;
		MS5837::MS5837Decode* ms5837_decoder;
		void decode_cb(const scion_types::msg::CanFrame::SharedPtr) const;
		
};

#endif
