#include "RemoteDR16Driver.hpp"
#include <memory.h>

void RemoteDR16::decode(uint8_t *buffer)
{
	memcpy(remote_buffer_, buffer, 18);
	// controller decode
	remote_.controller.right_stick.x = ((remote_buffer_[0] | (remote_buffer_[1] << 8)) & 0x07ff) - 1024;
	remote_.controller.right_stick.y = (((remote_buffer_[1] >> 3) | (remote_buffer_[2] << 5)) & 0x07ff) - 1024;
	remote_.controller.left_stick.x = (((remote_buffer_[2] >> 6) | (remote_buffer_[3] << 2) | (remote_buffer_[4] << 10)) & 0x07ff) - 1024;
	remote_.controller.left_stick.y = (((remote_buffer_[4] >> 1) | (remote_buffer_[5] << 7)) & 0x07ff) - 1024;
	
	remote_.controller.wheel = ((remote_buffer_[16] | (remote_buffer_[17] << 8)) & 0x07FF) - 1024;
	remote_.controller.left_switch = ((remote_buffer_[5] >> 4) & 0x000C) >> 2;
	remote_.controller.right_switch = ((remote_buffer_[5] >> 4) & 0x0003);

	// mouse decode
	remote_.mouse.x = (remote_buffer_[6]) | (remote_buffer_[7] << 8);
	remote_.mouse.y = remote_buffer_[8] | (remote_buffer_[9] << 8);
	remote_.mouse.z = remote_buffer_[10] | (remote_buffer_[11] << 8);
	remote_.mouse.left = remote_buffer_[12];
	remote_.mouse.right = remote_buffer_[13];

	// key decode
	uint16_t key_buffer = remote_buffer_[14] | (remote_buffer_[15] << 8);
	remote_.keyboard.W = (key_buffer >> 0) & 0x001;
	remote_.keyboard.S = (key_buffer >> 1) & 0x001;
	remote_.keyboard.A = (key_buffer >> 2) & 0x001;
	remote_.keyboard.D = (key_buffer >> 3) & 0x001;
	remote_.keyboard.Shift = (key_buffer >> 4) & 0x001;
	remote_.keyboard.Ctrl = (key_buffer >> 5) & 0x001;
	remote_.keyboard.Q = (key_buffer >> 6) & 0x001;
	remote_.keyboard.E = (key_buffer >> 7) & 0x001;
	remote_.keyboard.R = (key_buffer >> 8) & 0x001;
	remote_.keyboard.F = (key_buffer >> 9) & 0x001;
	remote_.keyboard.G = (key_buffer >> 10) & 0x001;
	remote_.keyboard.Z = (key_buffer >> 11) & 0x001;
	remote_.keyboard.X = (key_buffer >> 12) & 0x001;
	remote_.keyboard.C = (key_buffer >> 13) & 0x001;
	remote_.keyboard.V = (key_buffer >> 14) & 0x001;
	remote_.keyboard.B = (key_buffer >> 15) & 0x001;

	remote_.online_flag = 1;
}

void RemoteDR16::get_msg(prm_interfaces::msg::RemoteDR16Data &msg)
{
	if ((remote_.controller.left_stick.x < -REMOTE_STICK_MAX) || (remote_.controller.left_stick.x > REMOTE_STICK_MAX))
	{
		msg.left_joystick_x = 0.0f;
		msg.left_joystick_y = 0.0f;
		msg.right_joystick_x = 0.0f;
		msg.right_joystick_y = 0.0f;
		msg.dial_wheel = 0.0f;
		msg.left_switch = 2;
		msg.right_switch = 2;
		msg.mouse_x = 0.0f;
		msg.mouse_y = 0.0f;
		msg.mouse_z = 0.0f;
		msg.mouse_left = 0;
		msg.mouse_right = 0;
		msg.pressed_w = 0;
		msg.pressed_s = 0;
		msg.pressed_a = 0;
		msg.pressed_d = 0;
		msg.pressed_q = 0;
		msg.pressed_e = 0;
		msg.pressed_r = 0;
		msg.pressed_f = 0;
		msg.pressed_g = 0;
		msg.pressed_z = 0;
		msg.pressed_x = 0;
		msg.pressed_c = 0;
		msg.pressed_v = 0;
		msg.pressed_b = 0;
		msg.online_flag = 0;
		return;
	}
	msg.left_joystick_x = static_cast<float>(remote_.controller.left_stick.x) / REMOTE_STICK_MAX;
	msg.left_joystick_y = static_cast<float>(remote_.controller.left_stick.y) / REMOTE_STICK_MAX;
	msg.right_joystick_x = static_cast<float>(remote_.controller.right_stick.x) / REMOTE_STICK_MAX;
	msg.right_joystick_y = static_cast<float>(remote_.controller.right_stick.y) / REMOTE_STICK_MAX;
	msg.dial_wheel = static_cast<float>(remote_.controller.wheel);
	msg.left_switch = remote_.controller.left_switch;
	msg.right_switch = remote_.controller.right_switch;
	msg.mouse_x = static_cast<float>(remote_.mouse.x) / MOUSE_SPEED_MAX;
	msg.mouse_y = static_cast<float>(remote_.mouse.y) / MOUSE_SPEED_MAX;
	msg.mouse_z = static_cast<float>(remote_.mouse.z) / MOUSE_SPEED_MAX;
	msg.mouse_left = remote_.mouse.left;
	msg.mouse_right = remote_.mouse.right;
	msg.pressed_w = remote_.keyboard.W;
	msg.pressed_s = remote_.keyboard.S;
	msg.pressed_a = remote_.keyboard.A;
	msg.pressed_d = remote_.keyboard.D;
	msg.pressed_q = remote_.keyboard.Q;
	msg.pressed_e = remote_.keyboard.E;
	msg.pressed_r = remote_.keyboard.R;
	msg.pressed_f = remote_.keyboard.F;
	msg.pressed_g = remote_.keyboard.G;
	msg.pressed_z = remote_.keyboard.Z;
	msg.pressed_x = remote_.keyboard.X;
	msg.pressed_c = remote_.keyboard.C;
	msg.pressed_v = remote_.keyboard.V;
	msg.pressed_b = remote_.keyboard.B;
	msg.online_flag = remote_.online_flag;
	return;
}
