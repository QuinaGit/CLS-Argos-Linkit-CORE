#include <sstream>
#include <iostream>

#include "dte_handler.hpp"
#include "config_store_fs.hpp"
#include "fake_memory_access.hpp"
#include "fake_battery_mon.hpp"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "mock_logger.hpp"
#include "previpass.h"

#define BLOCK_COUNT   (256)
#define BLOCK_SIZE    (64*1024)
#define PAGE_SIZE     (256)
#define MAX_FILE_SIZE (4*1024*1024)


extern FileSystem *main_filesystem;
extern ConfigurationStore *configuration_store;
extern MemoryAccess *memory_access;
extern Logger *sensor_log;
extern Logger *system_log;
extern BatteryMonitor *battery_monitor;

TEST_GROUP(DTEHandler)
{
	RamFlash *ram_flash;
	LFSFileSystem *ram_filesystem;
	LFSConfigurationStore *store;
	FakeMemoryAccess *fake_memory_access;
	MockLog *mock_system_log;
	MockLog *mock_sensor_log;
	DTEHandler *dte_handler;
	FakeBatteryMonitor *fake_battery_monitor;

	void setup() {
		ram_flash = new RamFlash(BLOCK_COUNT, BLOCK_SIZE, PAGE_SIZE);
		ram_filesystem = new LFSFileSystem(ram_flash);
		ram_filesystem->format();
		ram_filesystem->mount();
		main_filesystem = ram_filesystem;
		store = new LFSConfigurationStore(*ram_filesystem);
		store->init();
		configuration_store = store;
		fake_memory_access = new FakeMemoryAccess();
		memory_access = fake_memory_access;
		mock_system_log = new MockLog;
		system_log = mock_system_log;
		mock_sensor_log = new MockLog;
		sensor_log = mock_sensor_log;
		dte_handler = new DTEHandler();
		fake_battery_monitor = new FakeBatteryMonitor();
		battery_monitor = fake_battery_monitor;
		fake_battery_monitor->m_level = 0U;
		fake_battery_monitor->m_voltage = 0U;
	}

	void teardown() {
		delete dte_handler;
		delete mock_sensor_log;
		delete mock_system_log;
		delete fake_memory_access;
		delete fake_battery_monitor;
		delete store;
		ram_filesystem->umount();
		delete ram_filesystem;
	}
};


TEST(DTEHandler, PARML_REQ)
{
	std::string resp;
	std::string req = DTEEncoder::encode(DTECommand::PARML_REQ);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARML#113;IDT06,IDT07,IDT02,IDT03,ART01,ART02,POT03,POT05,IDP11,ART03,ARP03,ARP04,ARP05,ARP01,ARP19,ARP18,GNP01,ARP11,ARP16,GNP02,GNP03,GNP05,UNP01,UNP02,UNP03,LBP01,LBP02,LBP03,ARP06,LBP04,LBP05,LBP06,ARP12,LBP07,LBP08,LBP09,UNP04,PPP01,PPP02,PPP03,PPP04,PPP05,PPP06,GNP09,GNP10,GNP11\r", resp.c_str());
}

TEST(DTEHandler, PARMW_REQ)
{
	std::string resp;
	std::string req = "$PARMW#007;ARP04=4\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());
	CHECK_TRUE(BaseArgosPower::POWER_500_MW == configuration_store->read_param<BaseArgosPower>(ParamID::ARGOS_POWER));
}

TEST(DTEHandler, PARMR_REQ)
{
	std::string resp;
	std::string req = "$PARMR#0D7;IDT06,IDT07,IDT02,IDT03,ART01,ART02,POT03,POT05,IDP11,ART03,ARP03,ARP04,ARP05,ARP01,ARP19,ARP18,GNP01,ARP11,ARP16,GNP02,GNP03,GNP05,UNP01,UNP02,UNP03,LBP01,LBP02,LBP03,ARP06,LBP04,LBP05,LBP06,ARP12,LBP07,LBP08,LBP09\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMR#175;IDT06=0,IDT07=0,IDT02=SURFACEBOX,IDT03=V0.1,ART01=Thu Jan  1 00:00:00 1970,ART02=0,POT03=0,POT05=Thu Jan  1 00:00:00 1970,IDP11=,ART03=Thu Jan  1 00:00:00 1970,ARP03=399,ARP04=4,ARP05=45,ARP01=0,ARP19=1,ARP18=0,GNP01=0,ARP11=1,ARP16=1,GNP02=0,GNP03=2,GNP05=60,UNP01=0,UNP02=1,UNP03=1,LBP01=0,LBP02=0,LBP03=4,ARP06=45,LBP04=0,LBP05=0,LBP06=0,ARP12=1,LBP07=2,LBP08=1,LBP09=60\r", resp.c_str());
}

TEST(DTEHandler, STATR_REQ)
{
	std::string resp;
	std::string req = "$STATR#0D7;IDT06,IDT07,IDT02,IDT03,ART01,ART02,POT03,POT05,IDP11,ART03,ARP03,ARP04,ARP05,ARP01,ARP19,ARP18,GNP01,ARP11,ARP16,GNP02,GNP03,GNP05,UNP01,UNP02,UNP03,LBP01,LBP02,LBP03,ARP06,LBP04,LBP05,LBP06,ARP12,LBP07,LBP08,LBP09\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;STATR#175;IDT06=0,IDT07=0,IDT02=SURFACEBOX,IDT03=V0.1,ART01=Thu Jan  1 00:00:00 1970,ART02=0,POT03=0,POT05=Thu Jan  1 00:00:00 1970,IDP11=,ART03=Thu Jan  1 00:00:00 1970,ARP03=399,ARP04=4,ARP05=45,ARP01=0,ARP19=1,ARP18=0,GNP01=0,ARP11=1,ARP16=1,GNP02=0,GNP03=2,GNP05=60,UNP01=0,UNP02=1,UNP03=1,LBP01=0,LBP02=0,LBP03=4,ARP06=45,LBP04=0,LBP05=0,LBP06=0,ARP12=1,LBP07=2,LBP08=1,LBP09=60\r", resp.c_str());
}

TEST(DTEHandler, STATR_REQ_CheckEmptyRequest)
{
	std::string resp;
	std::string req = "$STATR#000;\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;STATR#098;IDT06=0,IDT07=0,IDT02=SURFACEBOX,IDT03=V0.1,ART01=Thu Jan  1 00:00:00 1970,ART02=0,POT03=0,POT05=Thu Jan  1 00:00:00 1970,ART03=Thu Jan  1 00:00:00 1970\r", resp.c_str());
}

TEST(DTEHandler, PARMR_REQ_CheckEmptyRequest)
{
	std::string resp;
	std::string req = "$PARMR#000;\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMR#136;IDP11=,ARP03=399,ARP04=4,ARP05=45,ARP01=0,ARP19=1,ARP18=0,GNP01=0,ARP11=1,ARP16=1,GNP02=0,GNP03=2,GNP05=60,UNP01=0,UNP02=1,UNP03=1,LBP01=0,LBP02=0,LBP03=4,ARP06=45,LBP04=0,LBP05=0,LBP06=0,ARP12=1,LBP07=2,LBP08=1,LBP09=60,UNP04=1,PPP01=5,PPP02=90,PPP03=300,PPP04=1000,PPP05=300,PPP06=10,GNP09=60,GNP10=1,GNP11=5\r", resp.c_str());
}

TEST(DTEHandler, PROFW_PROFR_REQ)
{
	std::string resp;
	std::string req = "$PROFW#018;Profile Name For Tracker\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PROFW#000;\r", resp.c_str());
	req = "$PROFR#000;\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PROFR#018;Profile Name For Tracker\r", resp.c_str());
}

TEST(DTEHandler, SECUR_REQ)
{
	std::string resp;
	std::string req = "$SECUR#004;1234\r";
	CHECK_TRUE(DTEAction::SECUR == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;SECUR#000;\r", resp.c_str());
}

TEST(DTEHandler, RSTVW_REQ)
{
	unsigned int tx_counter;

	// Increment TX_COUNTER to 1
	configuration_store->increment_tx_counter();
	tx_counter = configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER);
	CHECK_EQUAL(1U, tx_counter);

	// This should reset TX_COUNTER to zero
	std::string resp;
	std::string req = "$RSTVW#001;1\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;RSTVW#000;\r", resp.c_str());

	tx_counter = configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER);
	CHECK_EQUAL(0U, tx_counter);
}

TEST(DTEHandler, RSTBW_REQ)
{
	std::string resp;
	std::string req = "$RSTBW#000;\r";
	CHECK_TRUE(DTEAction::RESET == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;RSTBW#000;\r", resp.c_str());
}

TEST(DTEHandler, FACTW_REQ)
{
	std::string resp;
	std::string req = "$FACTW#000;\r";
	CHECK_TRUE(DTEAction::FACTR == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;FACTW#000;\r", resp.c_str());
}

TEST(DTEHandler, DUMPM_REQ)
{
	std::string resp;
	std::string req = "$DUMPM#007;100,200\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;DUMPM#2AC;AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8gISIjJCUmJygpKissLS4vMDEyMzQ1Njc4OTo7PD0+P0BBQkNERUZHSElKS0xNTk9QUVJTVFVWV1hZWltcXV5fYGFiY2RlZmdoaWprbG1ub3BxcnN0dXZ3eHl6e3x9fn+AgYKDhIWGh4iJiouMjY6PkJGSk5SVlpeYmZqbnJ2en6ChoqOkpaanqKmqq6ytrq+wsbKztLW2t7i5uru8vb6/wMHCw8TFxsfIycrLzM3Oz9DR0tPU1dbX2Nna29zd3t/g4eLj5OXm5+jp6uvs7e7v8PHy8/T19vf4+fr7/P3+/wABAgMEBQYHCAkKCwwNDg8QERITFBUWFxgZGhscHR4fICEiIyQlJicoKSorLC0uLzAxMjM0NTY3ODk6Ozw9Pj9AQUJDREVGR0hJSktMTU5PUFFSU1RVVldYWVpbXF1eX2BhYmNkZWZnaGlqa2xtbm9wcXJzdHV2d3h5ent8fX5/gIGCg4SFhoeIiYqLjI2Oj5CRkpOUlZaXmJmam5ydnp+goaKjpKWmp6ipqqusra6vsLGys7S1tre4ubq7vL2+v8DBwsPExcbHyMnKy8zNzs/Q0dLT1NXW19jZ2tvc3d7f4OHi4+Tl5ufo6err7O3u7/Dx8vP09fb3+Pn6+/z9/v8=\r", resp.c_str());
}

TEST(DTEHandler, ZONEW_REQ)
{
	BaseRawData zone_raw = {0,0, ""};
	BaseZone zone;
	zone.zone_id = 1;
	zone.zone_type = BaseZoneType::CIRCLE;
	zone.enable_monitoring = true;
	zone.enable_entering_leaving_events = true;
	zone.enable_out_of_zone_detection_mode = true;
	zone.enable_activation_date = true;
	zone.year = 1970;
	zone.month = 1;
	zone.day = 1;
	zone.hour = 0;
	zone.minute = 0;
	zone.comms_vector = BaseCommsVector::ARGOS_PREFERRED;
	zone.delta_arg_loc_argos_seconds = 7*60U;
	zone.delta_arg_loc_cellular_seconds = 0;
	zone.argos_extra_flags_enable = true;
	zone.argos_depth_pile = BaseArgosDepthPile::DEPTH_PILE_1;
	zone.argos_power = BaseArgosPower::POWER_500_MW;
	zone.argos_time_repetition_seconds = 60U;
	zone.argos_mode = BaseArgosMode::DUTY_CYCLE;
	zone.argos_duty_cycle = 0b101010101010101010101010;
	zone.gnss_extra_flags_enable = true;
	zone.hdop_filter_threshold = 2U;
	zone.gnss_acquisition_timeout_seconds = 60U;
	zone.center_latitude_y = 0;
	zone.center_longitude_x = 0;
	zone.radius_m = 0;
	ZoneCodec::encode(zone, zone_raw.str);

	std::string resp;
	std::string req = DTEEncoder::encode(DTECommand::ZONEW_REQ, zone_raw);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;ZONEW#000;\r", resp.c_str());

	BaseZone& stored_zone = configuration_store->read_zone();
	CHECK_TRUE(stored_zone == zone);
}

TEST(DTEHandler, ZONER_REQ)
{
	BaseZone zone;
	zone.zone_id = 1;
	zone.zone_type = BaseZoneType::CIRCLE;
	zone.enable_monitoring = true;
	zone.enable_entering_leaving_events = true;
	zone.enable_out_of_zone_detection_mode = true;
	zone.enable_activation_date = true;
	zone.year = 1970;
	zone.month = 1;
	zone.day = 1;
	zone.hour = 0;
	zone.minute = 0;
	zone.comms_vector = BaseCommsVector::ARGOS_PREFERRED;
	zone.delta_arg_loc_argos_seconds = 7*60U;
	zone.delta_arg_loc_cellular_seconds = 0;
	zone.argos_extra_flags_enable = true;
	zone.argos_depth_pile = BaseArgosDepthPile::DEPTH_PILE_1;
	zone.argos_power = BaseArgosPower::POWER_500_MW;
	zone.argos_time_repetition_seconds = 60U;
	zone.argos_mode = BaseArgosMode::DUTY_CYCLE;
	zone.argos_duty_cycle = 0b101010101010101010101010;
	zone.gnss_extra_flags_enable = true;
	zone.hdop_filter_threshold = 2U;
	zone.gnss_acquisition_timeout_seconds = 60U;
	zone.center_latitude_y = 0;
	zone.center_longitude_x = 0;
	zone.radius_m = 0;
	configuration_store->write_zone(zone);

	std::string resp;
	std::string req = DTEEncoder::encode(DTECommand::ZONER_REQ, 1);

	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	CHECK_EQUAL("$O;ZONER#01C;A/cIQAIgI4bqqqqkeNu6Abd0AAA=\r"s, resp);
	std::string zone_resp_bits = websocketpp::base64_decode("A/cIQAIgI4bqqqqkeNu6Abd0AAA="s);
	BaseZone zone_resp_decoded;
	ZoneCodec::decode(zone_resp_bits, zone_resp_decoded);
	CHECK_TRUE(zone == zone_resp_decoded);
}

TEST(DTEHandler, PASPW_REQ)
{
	// Supplied by CLS
	std::string allcast_ref = "00000BE5008480208895C628AFD3EADAD37342125049EDF300000BE500C48020889505625F8BE8DB23750B1355B0FFEA00000BE500A4802088800A69B42D26C6BAFBFA003BEF619A00000BE50094802088C55528BAF528C6CAFC5E0042864CE600000BE500B480208889D26A39B528C6BAFC0D0042CB5A7F00000BE500548020888014E6BB3DCABCCAC1241143642DE100000BE500D480208895960CC7EE9CAF7A720F003C2126DC00000C75008603A5C900B7C500800C00D4CE845000005F5006607A58900B78C00D484741";
	std::string allcast_binary;

	// Transcode to binary
	for (unsigned int i = 0; i < allcast_ref.length(); i += 2) {
		int byte;
		std::stringstream converter;
		converter << std::hex << allcast_ref.substr(i, 2);
		converter >> byte;
		allcast_binary.append(1, (unsigned char)byte & 0xFF);
	}

	BaseRawData paspw_raw = {0,0, allcast_binary };

	std::string resp;
	std::string req = DTEEncoder::encode(DTECommand::PASPW_REQ, paspw_raw);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PASPW#000;\r", resp.c_str());

	BasePassPredict& pass_predict = configuration_store->read_pass_predict();

	std::time_t actual_aop_date = configuration_store->read_param<std::time_t>(ParamID::ARGOS_AOP_DATE);
	std::time_t expected_aop_date = convert_epochtime(pass_predict.records[0].bulletin.year, pass_predict.records[0].bulletin.month, pass_predict.records[0].bulletin.day, pass_predict.records[0].bulletin.hour, pass_predict.records[0].bulletin.minute, pass_predict.records[0].bulletin.second);

	CHECK_EQUAL(expected_aop_date, actual_aop_date);
}

TEST(DTEHandler, PASPW_REQ2)
{
	// Supplied by CLS
	std::string allcast_ref = "00000BE500848418088455EB03DBECDAAB7238094E71CF2F00000BE500C48418089189A44D1BEADAFB744404558E604A00000BE500A4841808C90D263F8D26C6BAFBFF003A1337B000000BE5009484180896012944352AC6B2FBD1004228CCA900000BE500B484180884122AC2C528C6BAFC11004271384600000BE500548418088E4DE5E76DCABCBAC0BB02421E062600000BE500D48418088648CDE4469EAF5A7160033C34E25C00000C75008603A5C900B7C500800C00D4CE845000005F5006607A58900B78C00D48474100000BE700848418088455EB03DBECDAAB7238094E71633000000BE700C48418089189A44D1BEADAFB744404558ECC5500000BE700A4841808C90D263F8D26C6BAFBFF003A139BAF00000BE7009484180896012944352AC6B2FBD100422860B600000BE700B484180884122AC2C528C6BAFC11004271945900000BE700548418088E4DE5E76DCABCBAC0BB02421EAA3900000BE700D48418088648CDE4469EAF5A7160033C344E4300000C77008603A5C900B7C500800C00D4C758A000005F7006607A58900B78C00D48ED3B00000BE400848418088455EB03DBECDAAB7238094E71113000000BE400C48418089189A44D1BEADAFB744404558EBE5500000BE400A4841808C90D263F8D26C6BAFBFF003A13E9AF00000BE4009484180896012944352AC6B2FBD100422812B600000BE400B484180884122AC2C528C6BAFC11004271E65900000BE400548418088E4DE5E76DCABCBAC0BB02421ED83900000BE400D48418088648CDE4469EAF5A7160033C343C4300000C74008603A5C900B7C500800C00D4C2EB2000005F4006607A58900B78C00D48127C";
	std::string allcast_binary;

	// Transcode to binary
	for (unsigned int i = 0; i < allcast_ref.length(); i += 2) {
		int byte;
		std::stringstream converter;
		converter << std::hex << allcast_ref.substr(i, 2);
		converter >> byte;
		allcast_binary.append(1, (unsigned char)byte & 0xFF);
	}

	BaseRawData paspw_raw = {0,0, allcast_binary };

	std::string resp;
	std::string req = DTEEncoder::encode(DTECommand::PASPW_REQ, paspw_raw);
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PASPW#000;\r", resp.c_str());

	BasePassPredict& stored_pass_predict = configuration_store->read_pass_predict();

#if 0
	for (unsigned int i = 0; i < stored_pass_predict.num_records; i++) {
		DEBUG_TRACE("paspw[%u].satDcsId=%01x", i, stored_pass_predict.records[i].satDcsId);
		DEBUG_TRACE("paspw[%u].satHexId=%01x", i, stored_pass_predict.records[i].satHexId);
		DEBUG_TRACE("paspw[%u].uplinkStatus=%u", i, stored_pass_predict.records[i].uplinkStatus);
		DEBUG_TRACE("paspw[%u].year=%u", i, stored_pass_predict.records[i].bulletin.year);
		DEBUG_TRACE("paspw[%u].month=%u", i, stored_pass_predict.records[i].bulletin.month);
		DEBUG_TRACE("paspw[%u].day=%u", i, stored_pass_predict.records[i].bulletin.day);
		DEBUG_TRACE("paspw[%u].hour=%u", i, stored_pass_predict.records[i].bulletin.hour);
		DEBUG_TRACE("paspw[%u].minute=%u", i, stored_pass_predict.records[i].bulletin.minute);
		DEBUG_TRACE("paspw[%u].second=%u", i, stored_pass_predict.records[i].bulletin.second);
		DEBUG_TRACE("paspw[%u].semiMajorAxisKm=%f", i, (double)stored_pass_predict.records[i].semiMajorAxisKm);
		DEBUG_TRACE("paspw[%u].inclinationDeg=%f", i, (double)stored_pass_predict.records[i].inclinationDeg);
		DEBUG_TRACE("paspw[%u].ascNodeLongitudeDeg=%f", i, (double)stored_pass_predict.records[i].ascNodeLongitudeDeg);
		DEBUG_TRACE("paspw[%u].ascNodeDriftDeg=%f", i, (double)stored_pass_predict.records[i].ascNodeDriftDeg);
		DEBUG_TRACE("paspw[%u].orbitPeriodMin=%f", i, (double)stored_pass_predict.records[i].orbitPeriodMin);
		DEBUG_TRACE("paspw[%u].semiMajorAxisDriftMeterPerDay=%f", i, (double)stored_pass_predict.records[i].semiMajorAxisDriftMeterPerDay);
	}
#endif

	// Get PREVIPASS results using every minute of day as start of search
	std::time_t last_epoch = 0;

	for (unsigned int minute_of_day = 0; minute_of_day < 1440; minute_of_day += 1) {
		struct PredictionPassConfiguration_t prepasConfiguration = {

			51.3764385f,                       //< Geodetic latitude of the beacon (deg.) [-90, 90]
			-2.1182383f,                       //< Geodetic longitude of the beacon (deg.E)[0, 360]
			{ 2021, 3, 8, (uint8_t)(minute_of_day/60), (uint8_t)(minute_of_day%60), 0 },  //< Beginning of prediction (Y/M/D, hh:mm:ss)
			{ 2021, 3, 10, 0, 0, 0 },  //< End of prediction (Y/M/D, hh:mm:ss)
			60.0f,                         //< Minimum elevation of passes [0, 90](default 5 deg)
			90.0f,                        //< Maximum elevation of passes  [maxElevation >=
										  //< minElevation] (default 90 deg)
			1.0f,                         //< Minimum duration (default 5 minutes)
			1000,                         //< Maximum number of passes per satellite (default
										  //< 1000)
			5,                            //< Linear time margin (in minutes/6months) (default
										  //< 5 minutes/6months)
			10                            //< Computation step (default 30s)
		};

		uint8_t nbSatsInAopTable = stored_pass_predict.num_records;
		SatelliteNextPassPrediction_t nextPass;
		if (PREVIPASS_compute_next_pass(
				&prepasConfiguration,
				stored_pass_predict.records,
				nbSatsInAopTable,
				&nextPass)) {
			std::time_t t = nextPass.epoch;
			//std::cout << std::setw(2) << std::setfill('0') << (minute_of_day/60) << ":" << std::setw(2) << std::setfill('0') << (minute_of_day%60) << ", " << std::put_time(std::gmtime(&t), "%c") << std::endl;
			CHECK_TRUE(last_epoch <= (std::time_t)t);
			last_epoch = t;
		} else {
			std::cout << std::setw(2) << std::setfill('0') << (minute_of_day/60) << ":" << std::setw(2) << std::setfill('0') << (minute_of_day%60) << ", " << "no result" << std::endl;
			CHECK_TRUE(false);
		}
	}
}

TEST(DTEHandler, DUMPD_REQ)
{
	std::string req = DTEEncoder::encode(DTECommand::DUMPD_REQ, BaseLogDType::SENSOR);
	std::string resp;

	mock().expectOneCall("num_entries").onObject(mock_sensor_log).andReturnValue(1);
	mock().expectOneCall("read").onObject(mock_sensor_log).withIntParameter("index", 0).ignoreOtherParameters();

	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;DUMPD#010;0,0,AAAAAAAAAAAA\r", resp.c_str());

	// Check N entries are retrieved requiring two passes
	mock().expectOneCall("num_entries").onObject(mock_sensor_log).andReturnValue(20);
	for (unsigned int i = 0; i < 16; i++)
		mock().expectOneCall("read").onObject(mock_sensor_log).withIntParameter("index", i).ignoreOtherParameters();
	CHECK_TRUE(DTEAction::AGAIN == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;DUMPD#0C4;0,1,AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\r", resp.c_str());

	mock().expectOneCall("num_entries").onObject(mock_sensor_log).andReturnValue(20);
	for (unsigned int i = 16; i < 20; i++)
		mock().expectOneCall("read").onObject(mock_sensor_log).withIntParameter("index", i).ignoreOtherParameters();
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;DUMPD#034;1,1,AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\r", resp.c_str());

	mock().checkExpectations();
}

TEST(DTEHandler, DUMPD_REQ_EmptyLogFile)
{
	std::string req = DTEEncoder::encode(DTECommand::DUMPD_REQ, BaseLogDType::SENSOR);
	std::string resp;

	mock().expectOneCall("num_entries").onObject(mock_sensor_log).andReturnValue(0);

	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;DUMPD#004;0,0,\r", resp.c_str());

	mock().checkExpectations();
}

TEST(DTEHandler, WritingReadOnlyAttributesIsIgnored)
{
	std::string resp;
	std::string req = "$PARMW#007;ART02=1\r";
	CHECK_TRUE(DTEAction::CONFIG_UPDATED == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$O;PARMW#000;\r", resp.c_str());

	unsigned int tx_counter = configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER);

	CHECK_EQUAL(0, tx_counter);
}

TEST(DTEHandler, WritingOutOfRangeValue)
{
	std::string resp;
	std::string req = "$PARMW#009;PPP01=-12\r";
	CHECK_TRUE(DTEAction::NONE == dte_handler->handle_dte_message(req, resp));
	STRCMP_EQUAL("$N;PARMW#001;5\r", resp.c_str());
}
