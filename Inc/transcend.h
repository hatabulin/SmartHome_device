typedef struct RemoteCodes
{
	uint32_t code;
    char* 	 name;
} RemoteCodes;

#define IR_REMOTE_COMMANDS_MAX 19
const RemoteCodes IR_Remote[IR_REMOTE_COMMANDS_MAX] = {
		{ 0xf17e2757,"TRANSCEND_HOME" },
		{ 0x9ec7c88f,"TRANSCEND_CALENDAR" },
		{ 0xa7b76553, "TRANSCEND_POWER_OFF" },
		{ 0x865adf1b, "TRANSCEND_STOP" },
		{ 0xde260593, "TRANSCEND_PREV" },
		{ 0x37d35d97, "TRANSCEND_NEXT" },
		{ 0xda5ad0b3, "TRANSCEND_PLAY" },
		{ 0x41285df7, "TRANSCEND_VOL_UP" },
		{ 0x8bd34f2f, "TRANSCEND_VOL_DOWN" },
		{ 0x473de9f3, "TRANSCEND_OK" },
		{ 0x7cb889b3, "TRANSCEND_MODE" },
		{ 0x1c47af6f, "TRANSCEND_EXIT" },
		{ 0xe77b05f3, "TRANSCEND_ROTATE" },
		{ 0xf11a7ef3, "TRANSCEND_PHOTO" },
		{ 0x4ac7d6f7, "TRANSCEND_MUSIC" },
		{ 0x9ec7c88f, "TRANSCEND_CALENDAR" },
		{ 0x204a3073, "TRANSCEND_SETTINGS" },
		{ 0x7cb889b3, "TRANSCEND_MODE" },
		{ 0x890e8e8f, "TRANSCEND_MUTE" }
};

#define TR_HOME			0xf17e2757
#define TR_CALENDAR		0x9ec7c88f
#define TR_POWER_OFF	0xa7b76553
#define TR_STOP			0x865adf1b
#define TR_PREV			0xde260593
#define TR_NEXT			0x37d35d97
#define TR_PLAY			0xda5ad0b3
#define TR_VOL_UP		0x41285df7
#define TR_VOL_DOWN		0x8bd34f2f
#define TR_OK			0x473de9f3
#define TR_MODE			0x7cb889b3
#define TR_EXIT			0x1c47af6f
#define TR_ROTATE		0xe77b05f3
#define TR_PHOTO		0xf11a7ef3
#define TR_MUSIC		0x4ac7d6f7
#define TR_CALENDAR		0x9ec7c88f
#define TR_SETTINGS		0x204a3073
#define TR_HOT_BUTTON1	0x204a3073
#define TR_MODE			0x7cb889b3
#define TR_MUTE			0x890e8e8f
